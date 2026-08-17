/*
 * Copyright (c) 2024 Monard2033
 * SPDX-License-Identifier: Apache-2.0
 *
 * Wireless keyboard receiver:
 *   ESB PRX -> validated keyboard/consumer input -> USB HID reports.
 */

#include <errno.h>
#include <string.h>

#include <esb.h>
#include <nrfx.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/class/hid.h>
#include <zephyr/usb/class/usb_hid.h>
#include <zephyr/usb/usb_device.h>

LOG_MODULE_REGISTER(receiver, LOG_LEVEL_INF);

#define INPUT_DATA_SIZE        8U
#define LINK_MAGIC             0xA5U
#define LINK_VERSION           0x02U
#define LINK_TYPE_KEYBOARD     0x01U
#define LINK_TYPE_CONSUMER     0x02U
#define LINK_RF_CHANNEL        80U
#define HID_QUEUE_DEPTH        32U
#define HID_REPORT_ID_KEYBOARD 0x01U
#define HID_REPORT_ID_CONSUMER 0x02U
#define KEYBOARD_LINK_TIMEOUT_MS 250U
#define HID_WRITE_TIMEOUT_MS     20U

/*
 * This is the on-air packet shared with the transmitter. ESB already adds a
 * CRC, acknowledgement, and retransmissions, so no application CRC is needed.
 */
struct link_input_packet {
	uint8_t magic;
	uint8_t version;
	uint8_t type;
	uint8_t sequence;
	uint8_t data[INPUT_DATA_SIZE];
} __packed;

BUILD_ASSERT(sizeof(struct link_input_packet) == 12U);

K_MSGQ_DEFINE(hid_report_queue, sizeof(struct link_input_packet),
	      HID_QUEUE_DEPTH, sizeof(uint32_t));
static K_SEM_DEFINE(hid_in_done, 0, 1);

/*
 * Report ID 1 is the normal 8-byte keyboard state. Report ID 2 is one
 * 16-bit Consumer Page usage (Play/Pause, volume, media navigation, etc.).
 */
static const uint8_t hid_report_descriptor[] = {
	0x05, 0x01,       /* Usage Page (Generic Desktop) */
	0x09, 0x06,       /* Usage (Keyboard) */
	0xA1, 0x01,       /* Collection (Application) */
	0x85, HID_REPORT_ID_KEYBOARD,
	0x05, 0x07,       /* Usage Page (Keyboard) */
	0x19, 0xE0,       /* Usage Minimum (Left Control) */
	0x29, 0xE7,       /* Usage Maximum (Right GUI) */
	0x15, 0x00,
	0x25, 0x01,
	0x75, 0x01,
	0x95, 0x08,
	0x81, 0x02,       /* Input (Data, Variable, Absolute) */
	0x75, 0x08,
	0x95, 0x01,
	0x81, 0x03,       /* Input (Constant) */
	0x05, 0x08,       /* Usage Page (LEDs) */
	0x19, 0x01,
	0x29, 0x05,
	0x75, 0x01,
	0x95, 0x05,
	0x91, 0x02,       /* Output (Data, Variable, Absolute) */
	0x75, 0x03,
	0x95, 0x01,
	0x91, 0x03,       /* Output (Constant) */
	0x05, 0x07,
	0x19, 0x00,
	0x29, 0x65,
	0x15, 0x00,
	0x25, 0x65,
	0x75, 0x08,
	0x95, 0x06,
	0x81, 0x00,       /* Input (Data, Array) */
	0xC0,

	0x05, 0x0C,       /* Usage Page (Consumer) */
	0x09, 0x01,       /* Usage (Consumer Control) */
	0xA1, 0x01,
	0x85, HID_REPORT_ID_CONSUMER,
	0x15, 0x00,
	0x26, 0xFF, 0x03,
	0x1A, 0x00, 0x00,
	0x2A, 0xFF, 0x03,
	0x75, 0x10,
	0x95, 0x01,
	0x81, 0x00,       /* Input (Data, Array, Absolute) */
	0xC0,
};
static const struct device *hid_device;
static atomic_t usb_configured;
static atomic_t usb_suspended;

static struct esb_payload esb_rx_payload;
static struct esb_config esb_config = ESB_DEFAULT_CONFIG;

static atomic_t radio_packets;
static atomic_t radio_bad_packets;
static atomic_t radio_duplicates;
static atomic_t radio_last_rssi;
static atomic_t radio_ready;
static atomic_t radio_init_error;
static atomic_t hid_queue_overruns;
static atomic_t hid_reports_sent;
static atomic_t hid_write_errors;
static atomic_t last_keyboard_packet_ms;
static atomic_t keyboard_watchdog_released;

/*
 * The ESB event handler runs in interrupt context. It may use a message queue
 * with K_NO_WAIT, but it must not call the USB HID endpoint API directly.
 */
static bool queue_hid_report(const struct link_input_packet *packet)
{
	if (k_msgq_put(&hid_report_queue, packet, K_NO_WAIT) == 0) {
		return true;
	}

	/*
	 * HID keyboard packets describe the complete current key state. If the
	 * consumer falls behind, purge stale transitions before retaining the
	 * newest keyboard state. A release must never be evicted by an older
	 * keyboard or Consumer Control transition.
	 */
	struct link_input_packet discarded;

	atomic_inc(&hid_queue_overruns);
	if (packet->type == LINK_TYPE_KEYBOARD) {
		k_msgq_purge(&hid_report_queue);
		return k_msgq_put(&hid_report_queue, packet, K_NO_WAIT) == 0;
	}
	if (k_msgq_get(&hid_report_queue, &discarded, K_NO_WAIT) == 0) {
		return k_msgq_put(&hid_report_queue, packet, K_NO_WAIT) == 0;
	}
	return false;
}

static void receiver_esb_event_handler(const struct esb_evt *event)
{
	static bool previous_keyboard_valid;
	static bool previous_consumer_valid;
	static uint8_t previous_keyboard[INPUT_DATA_SIZE];
	static uint8_t previous_consumer[INPUT_DATA_SIZE];

	if (event->evt_id != ESB_EVENT_RX_RECEIVED) {
		return;
	}

	while (esb_read_rx_payload(&esb_rx_payload) == 0) {
		struct link_input_packet packet;

		atomic_inc(&radio_packets);
		atomic_set(&radio_last_rssi, esb_rx_payload.rssi);
		if (esb_rx_payload.length != sizeof(packet)) {
			atomic_inc(&radio_bad_packets);
			continue;
		}

		memcpy(&packet, esb_rx_payload.data, sizeof(packet));
		if (packet.magic != LINK_MAGIC ||
		    packet.version != LINK_VERSION ||
		    (packet.type != LINK_TYPE_KEYBOARD &&
		     packet.type != LINK_TYPE_CONSUMER)) {
			atomic_inc(&radio_bad_packets);
			continue;
		}

		if (packet.type == LINK_TYPE_KEYBOARD) {
			atomic_set(&last_keyboard_packet_ms, k_uptime_get_32());
		}

		bool *previous_valid = packet.type == LINK_TYPE_KEYBOARD ?
			&previous_keyboard_valid : &previous_consumer_valid;
		uint8_t *previous_data = packet.type == LINK_TYPE_KEYBOARD ?
			previous_keyboard : previous_consumer;
		bool const watchdog_needs_state =
			packet.type == LINK_TYPE_KEYBOARD &&
			atomic_get(&keyboard_watchdog_released) != 0;

		/*
		 * Reports are complete states. Keepalives and RF retries do not
		 * need to enter the HID transition queue when the state is equal.
		 */
		if (*previous_valid && !watchdog_needs_state &&
		    memcmp(packet.data, previous_data,
			   INPUT_DATA_SIZE) == 0) {
			atomic_inc(&radio_duplicates);
			continue;
		}

		if (queue_hid_report(&packet)) {
			*previous_valid = true;
			memcpy(previous_data, packet.data, INPUT_DATA_SIZE);
			if (packet.type == LINK_TYPE_KEYBOARD) {
				atomic_set(&keyboard_watchdog_released, 0);
			}
		}
	}
}

static int esb_initialize(void)
{
	static const uint8_t base_address_0[4] = {
		0xE7, 0xE7, 0xE7, 0xE7
	};
	static const uint8_t base_address_1[4] = {
		0xC2, 0xC2, 0xC2, 0xC2
	};
	static const uint8_t address_prefixes[8] = {
		0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8
	};
	int err;

	esb_config.protocol = ESB_PROTOCOL_ESB_DPL;
	esb_config.mode = ESB_MODE_PRX;
	esb_config.bitrate = ESB_BITRATE_2MBPS;
	esb_config.tx_output_power = ESB_TX_POWER_8DBM;
	esb_config.payload_length = sizeof(struct link_input_packet);
	esb_config.selective_auto_ack = true;
	esb_config.use_fast_ramp_up = true;
	esb_config.event_handler = receiver_esb_event_handler;

	err = esb_init(&esb_config);
	if (err != 0) {
		return err;
	}

	err = esb_set_base_address_0(base_address_0);
	if (err != 0) {
		return err;
	}

	err = esb_set_base_address_1(base_address_1);
	if (err != 0) {
		return err;
	}

	err = esb_set_prefixes(address_prefixes, ARRAY_SIZE(address_prefixes));
	if (err != 0) {
		return err;
	}

	err = esb_set_rf_channel(LINK_RF_CHANNEL);
	if (err != 0) {
		return err;
	}

	return esb_start_rx();
}

static void hid_int_in_ready(const struct device *dev)
{
	ARG_UNUSED(dev);
	k_sem_give(&hid_in_done);
}

static const struct hid_ops hid_ops = {
	.int_in_ready = hid_int_in_ready,
};

static void usb_status_callback(enum usb_dc_status_code status,
				const uint8_t *param)
{
	ARG_UNUSED(param);

	switch (status) {
	case USB_DC_CONFIGURED:
		atomic_set(&usb_configured, 1);
		atomic_set(&usb_suspended, 0);
		break;
	case USB_DC_SUSPEND:
		atomic_set(&usb_suspended, 1);
		/* Release a sender if the host suspends with an IN pending. */
		k_sem_give(&hid_in_done);
		break;
	case USB_DC_RESUME:
		atomic_set(&usb_suspended, 0);
		break;
	case USB_DC_RESET:
	case USB_DC_DISCONNECTED:
		atomic_set(&usb_configured, 0);
		atomic_set(&usb_suspended, 0);
		/* Release a sender if the endpoint disappeared. */
		k_sem_give(&hid_in_done);
		break;
	default:
		break;
	}
}

static bool usb_hid_ready(void)
{
	return atomic_get(&usb_configured) != 0 &&
	       atomic_get(&usb_suspended) == 0;
}

static int send_hid_report(const uint8_t *report, size_t report_size)
{
	int err;

	if (!usb_hid_ready()) {
		return -ENOTCONN;
	}

	k_sem_reset(&hid_in_done);
	err = hid_int_ep_write(hid_device, report, report_size, NULL);
	if (err != 0) {
		atomic_inc(&hid_write_errors);
		return err;
	}

	/*
	 * Serialize endpoint writes. The USB callback also releases this wait
	 * on disconnect/suspend, preventing the bridge thread from hanging.
	 */
	err = k_sem_take(&hid_in_done, K_MSEC(HID_WRITE_TIMEOUT_MS));
	if (err != 0) {
		atomic_inc(&hid_write_errors);
		return -ETIMEDOUT;
	}
	if (!usb_hid_ready()) {
		return -ENOTCONN;
	}

	atomic_inc(&hid_reports_sent);
	return 0;
}

static int send_keyboard_report(const uint8_t data[INPUT_DATA_SIZE])
{
	uint8_t report[1U + INPUT_DATA_SIZE];

	report[0] = HID_REPORT_ID_KEYBOARD;
	memcpy(&report[1], data, INPUT_DATA_SIZE);
	return send_hid_report(report, sizeof(report));
}

static int send_consumer_report(const uint8_t data[INPUT_DATA_SIZE])
{
	const uint8_t report[] = {
		HID_REPORT_ID_CONSUMER,
		data[0],
		data[1],
	};

	return send_hid_report(report, sizeof(report));
}

static int usb_hid_initialize(void)
{
	int err;

	hid_device = device_get_binding("HID_0");
	if (hid_device == NULL) {
		return -ENODEV;
	}

	usb_hid_register_device(hid_device, hid_report_descriptor,
				sizeof(hid_report_descriptor), &hid_ops);

	err = usb_hid_init(hid_device);
	if (err != 0) {
		return err;
	}

	return usb_enable(usb_status_callback);
}

#if CONFIG_LOG
static void status_thread(void)
{
	for (;;) {
		k_sleep(K_SECONDS(5));
		LOG_INF("USB=%s ESB=%s init_err=%ld radio=%ld rssi=%ld bad=%ld dup=%ld "
			"queue_drop=%ld HID=%ld err=%ld "
			"RF[state=%lu cfg_ch=%u reg_ch=%lu mode=%lu pipes=0x%02lx] "
			"HF=0x%08lx",
			usb_hid_ready() ? "ready" : "offline",
			atomic_get(&radio_ready) != 0 ? "ready" : "offline",
			(long)atomic_get(&radio_init_error),
			(long)atomic_get(&radio_packets),
			(long)atomic_get(&radio_last_rssi),
			(long)atomic_get(&radio_bad_packets),
			(long)atomic_get(&radio_duplicates),
			(long)atomic_get(&hid_queue_overruns),
			(long)atomic_get(&hid_reports_sent),
			(long)atomic_get(&hid_write_errors),
			(unsigned long)NRF_RADIO->STATE,
			LINK_RF_CHANNEL,
			(unsigned long)NRF_RADIO->FREQUENCY,
			(unsigned long)NRF_RADIO->MODE,
			(unsigned long)NRF_RADIO->RXADDRESSES,
			(unsigned long)NRF_CLOCK->HFCLKSTAT);
	}
}

K_THREAD_DEFINE(status_thread_id, 1024, status_thread,
		NULL, NULL, NULL, 7, 0, 0);
#endif

static bool keyboard_data_is_released(const uint8_t data[INPUT_DATA_SIZE])
{
	static const uint8_t released[INPUT_DATA_SIZE] = { 0 };

	return memcmp(data, released, sizeof(released)) == 0;
}

int main(void)
{
	uint8_t current_keyboard[INPUT_DATA_SIZE] = { 0 };
	int err;

	LOG_INF("Starting ESB-to-USB keyboard and consumer-control receiver");

	/*
	 * Enumerate USB first. A radio initialization failure must never make
	 * the HID device disappear from the host.
	 */
	err = usb_hid_initialize();
	if (err != 0) {
		LOG_ERR("USB HID initialization failed: %d", err);
		return err;
	}

	err = esb_initialize();
	if (err != 0) {
		atomic_set(&radio_init_error, err);
		LOG_ERR("ESB initialization failed: %d; USB remains available", err);
	} else {
		atomic_set(&radio_ready, 1);
		LOG_INF("Ready: ESB PRX 2 Mbps channel %u, pipe 0 address "
			"E7:E7E7E7E7, USB keyboard + Consumer Control",
			LINK_RF_CHANNEL);
	}

	for (;;) {
		struct link_input_packet queued;

		if (!usb_hid_ready()) {
			/*
			 * Retain only keyboard state while the PC is absent. Consumer
			 * actions are transitions and must not be replayed later.
			 */
			while (k_msgq_get(&hid_report_queue, &queued,
					  K_NO_WAIT) == 0) {
				if (queued.type == LINK_TYPE_KEYBOARD) {
					memcpy(current_keyboard, queued.data,
					       sizeof(current_keyboard));
				}
			}
			k_msleep(1);
			continue;
		}

		if (!keyboard_data_is_released(current_keyboard) &&
		    (uint32_t)(k_uptime_get_32() -
		    (uint32_t)atomic_get(&last_keyboard_packet_ms)) >=
		    KEYBOARD_LINK_TIMEOUT_MS) {
			memset(current_keyboard, 0, sizeof(current_keyboard));
			atomic_set(&keyboard_watchdog_released, 1);
			(void)send_keyboard_report(current_keyboard);
			continue;
		}

		if (k_msgq_get(&hid_report_queue, &queued, K_NO_WAIT) == 0) {
			if (queued.type == LINK_TYPE_KEYBOARD) {
				memcpy(current_keyboard, queued.data,
				       sizeof(current_keyboard));
				(void)send_keyboard_report(current_keyboard);
			} else {
				(void)send_consumer_report(queued.data);
			}
		} else {
			/* Keyboard keepalive preserves the existing 1 kHz behavior. */
			(void)send_keyboard_report(current_keyboard);
		}
	}
}
