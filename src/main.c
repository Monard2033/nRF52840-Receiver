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
#define LINK_VERSION           0x03U
#define LINK_TYPE_KEYBOARD     0x01U
#define LINK_TYPE_CONSUMER     0x02U
#define LINK_TYPE_CONTROL      0x03U
#define LINK_TYPE_BATTERY      0x04U
#define LINK_CONTROL_POLL_ACK  0x02U
#define LINK_ACK_MAGIC         0x5AU
#define LINK_ACK_TYPE_LOCK_STATE 0x01U
#define LINK_RF_CHANNEL        80U
#define HID_QUEUE_DEPTH        128U
#define HID_REPORT_ID_KEYBOARD 0x01U
#define HID_REPORT_ID_CONSUMER 0x02U
#define HID_REPORT_ID_BATTERY  0x03U
#define HID_REPORT_ID_DFU      0x04U
#define HID_FEATURE_PAYLOAD_LEN 8U
#define HID_FEATURE_REPORT_LEN  (1U + HID_FEATURE_PAYLOAD_LEN)
#define DFU_FEATURE_PAYLOAD_LEN 8U
#define DFU_FEATURE_REPORT_LEN  (1U + DFU_FEATURE_PAYLOAD_LEN)
#define RELEASE_REPEAT_DELAY_MS  1U

#define LINK_TYPE_DFU_START     0x10U
#define LINK_TYPE_DFU_DATA      0x11U
#define LINK_TYPE_DFU_FINISH    0x12U
#define LINK_TYPE_DFU_STATUS    0x13U
#define LINK_ACK_TYPE_DFU       0x02U

#define DFU_STATUS_IDLE         0x00U
#define DFU_STATUS_BUSY         0x01U
#define DFU_STATUS_OK           0x02U
#define DFU_STATUS_ERR_SIZE     0x03U
#define DFU_STATUS_ERR_CRC      0x04U
#define DFU_STATUS_ERR_FLASH    0x05U
#define DFU_STATUS_SUCCESS      0x06U

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

struct link_ack_frame {
	uint8_t magic;
	uint8_t version;
	uint8_t type;
	uint8_t sequence;
	uint8_t data[INPUT_DATA_SIZE];
} __packed;

BUILD_ASSERT(sizeof(struct link_ack_frame) == 12U);

K_MSGQ_DEFINE(hid_report_queue, sizeof(struct link_input_packet),
	      HID_QUEUE_DEPTH, sizeof(uint32_t));
static K_SEM_DEFINE(hid_in_idle, 1, 1);

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

	/* Vendor-defined cached battery Feature/Input report (ID 3). */
	0x06, 0x00, 0xFF,
	0x09, 0x01,
	0xA1, 0x01,
	0x85, HID_REPORT_ID_BATTERY,
	0x15, 0x00,
	0x26, 0xFF, 0x00,
	0x75, 0x08,
	0x95, HID_FEATURE_PAYLOAD_LEN,
	0x19, 0x01,       /* Usage Minimum (vendor byte 1) */
	0x29, HID_FEATURE_PAYLOAD_LEN,
	0x81, 0x02,
	/* Local Usage items are cleared after each Main item. Repeat the range
	 * for the Feature fields so Windows can validate all eight variables. */
	0x19, 0x01,
	0x29, HID_FEATURE_PAYLOAD_LEN,
	0xB1, 0x02,
	0xC0,

	/* Vendor-defined DFU OTA Control/Data Feature report (ID 4). */
	0x06, 0x01, 0xFF,
	0x09, 0x01,
	0xA1, 0x01,
	0x85, HID_REPORT_ID_DFU,
	0x15, 0x00,
	0x26, 0xFF, 0x00,
	0x75, 0x08,
	0x95, DFU_FEATURE_PAYLOAD_LEN,
	0x19, 0x01,
	0x29, DFU_FEATURE_PAYLOAD_LEN,
	0xB1, 0x02,
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
static atomic_t keyboard_rx_generation;
static bool previous_keyboard_valid;
static bool previous_consumer_valid;
static bool previous_keyboard_queue_failed;
static bool previous_consumer_queue_failed;
static uint8_t previous_keyboard[INPUT_DATA_SIZE];
static uint8_t previous_consumer[INPUT_DATA_SIZE];
static uint8_t previous_keyboard_sequence;
static uint8_t previous_consumer_sequence;

static struct k_spinlock led_state_lock;
static uint8_t windows_led_state;
static uint8_t windows_led_sequence;
static uint8_t windows_led_epoch;
static bool windows_led_valid;
static atomic_t led_ack_pending;
static atomic_t led_ack_dirty;
static atomic_t battery_poll_requested;

static struct k_spinlock battery_cache_lock;
static uint8_t battery_percentage;
static uint8_t battery_state;
static uint16_t battery_millivolts;
static uint8_t battery_sequence;
static uint8_t battery_flags;
static uint32_t battery_last_update_ms;
static bool battery_cache_valid;

static struct k_spinlock dfu_state_lock;
static struct link_ack_frame dfu_pending_ack;
static bool dfu_ack_active;
static uint8_t dfu_current_status = DFU_STATUS_IDLE;
static uint8_t dfu_progress_percent = 0;
static uint16_t dfu_last_offset_div_4 = 0;
static uint8_t dfu_feature_report[DFU_FEATURE_REPORT_LEN];

/*
 * The ESB event handler runs in interrupt context. It may use a message queue
 * with K_NO_WAIT, but it must not call the USB HID endpoint API directly.
 */
static bool queue_hid_report(const struct link_input_packet *packet)
{
	if (k_msgq_put(&hid_report_queue, packet, K_NO_WAIT) == 0) {
		return true;
	}

	/* The normal path has enough room for more than 100 ms of 1 kHz input.
	 * If an external USB stall lasts longer than that, discard only the oldest
	 * frame; never purge the queue because that would destroy an ordered
	 * modifier/key/release sequence such as CTRL+C. */
	struct link_input_packet discarded;

	atomic_inc(&hid_queue_overruns);
	if (k_msgq_get(&hid_report_queue, &discarded, K_NO_WAIT) == 0) {
		return k_msgq_put(&hid_report_queue, packet, K_NO_WAIT) == 0;
	}
	return false;
}

static bool sequence_is_newer(uint8_t sequence, uint8_t previous)
{
	return (int8_t)(sequence - previous) > 0;
}

static bool battery_packet_is_valid(const struct link_input_packet *packet)
{
	return packet->data[0] <= 100U && packet->data[1] <= 4U &&
	       (packet->data[5] & 0x01U) != 0U &&
	       (packet->data[5] & 0xF0U) == 0U &&
	       packet->data[6] == 0U && packet->data[7] == 0U;
}

static void battery_cache_update(const struct link_input_packet *packet)
{
	k_spinlock_key_t key = k_spin_lock(&battery_cache_lock);
	bool const newer = !battery_cache_valid ||
		sequence_is_newer(packet->data[4], battery_sequence);

	if (newer) {
		battery_percentage = packet->data[0];
		battery_state = packet->data[1];
		battery_millivolts = (uint16_t)packet->data[2] |
			((uint16_t)packet->data[3] << 8);
		battery_sequence = packet->data[4];
		battery_flags = packet->data[5];
		battery_last_update_ms = k_uptime_get_32();
		battery_cache_valid = true;
	}
	k_spin_unlock(&battery_cache_lock, key);
}

static void receiver_set_led_state(uint8_t led_state)
{
	led_state &= 0x07U;
	k_spinlock_key_t key = k_spin_lock(&led_state_lock);
	bool const changed = !windows_led_valid ||
		windows_led_state != led_state;

	if (changed) {
		windows_led_state = led_state;
		windows_led_sequence++;
	}
	windows_led_valid = true;
	k_spin_unlock(&led_state_lock, key);

	if (changed) {
		atomic_set(&led_ack_dirty, 1);
		atomic_set(&led_ack_pending, 1);
	}
}

static bool receiver_queue_led_ack(void)
{
	struct link_ack_frame ack = {
		.magic = LINK_ACK_MAGIC,
		.version = LINK_VERSION,
		.type = LINK_ACK_TYPE_LOCK_STATE,
	};
	struct esb_payload payload = { 0 };
	bool battery_request_in_ack = false;

	k_spinlock_key_t dfu_key = k_spin_lock(&dfu_state_lock);
	if (dfu_ack_active) {
		ack = dfu_pending_ack;
		dfu_ack_active = false;
		k_spin_unlock(&dfu_state_lock, dfu_key);
	} else {
		k_spin_unlock(&dfu_state_lock, dfu_key);
		atomic_set(&led_ack_dirty, 0);
		battery_request_in_ack =
			atomic_cas(&battery_poll_requested, 1, 0);
		k_spinlock_key_t key = k_spin_lock(&led_state_lock);
		ack.sequence = windows_led_sequence;
		ack.data[0] = windows_led_state;
		ack.data[1] = (windows_led_valid ? 0x01U : 0x00U) |
			(battery_request_in_ack ? 0x02U : 0x00U);
		ack.data[2] = windows_led_epoch;
		k_spin_unlock(&led_state_lock, key);
	}

	payload.length = sizeof(ack);
	payload.pipe = 0;
	payload.noack = false;
	memcpy(payload.data, &ack, sizeof(ack));
	bool const queued = esb_write_payload(&payload) == 0;
	if (!queued) {
		/* No immediate retry: retain dirty state and wait for the next RX
		 * event, avoiding a tight loop on the three-entry ACK FIFO. */
		atomic_set(&led_ack_dirty, 1);
		if (battery_request_in_ack) {
			atomic_set(&battery_poll_requested, 1);
		}
	}
	return queued;
}

static void receiver_ack_task(void)
{
	if (atomic_cas(&led_ack_pending, 1, 0)) {
		(void)receiver_queue_led_ack();
	}
}

static void receiver_esb_event_handler(const struct esb_evt *event)
{
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

		if (packet.magic != LINK_MAGIC || packet.version != LINK_VERSION) {
			atomic_inc(&radio_bad_packets);
			continue;
		}

		if (atomic_get(&led_ack_dirty) != 0) {
			atomic_set(&led_ack_pending, 1);
		}

		if (packet.type == LINK_TYPE_DFU_STATUS) {
			k_spinlock_key_t key = k_spin_lock(&dfu_state_lock);
			dfu_current_status = packet.data[0];
			dfu_progress_percent = packet.data[1];
			dfu_last_offset_div_4 = (uint16_t)packet.data[2] |
				((uint16_t)packet.data[3] << 8);
			k_spin_unlock(&dfu_state_lock, key);
			atomic_set(&led_ack_pending, 1);
			continue;
		}

		if (packet.type == LINK_TYPE_BATTERY) {
			if (!battery_packet_is_valid(&packet)) {
				atomic_inc(&radio_bad_packets);
				continue;
			}
			battery_cache_update(&packet);
			continue;
		}

		if (packet.type == LINK_TYPE_CONTROL) {
			if (packet.data[0] != LINK_CONTROL_POLL_ACK) {
				atomic_inc(&radio_bad_packets);
				continue;
			}
			continue;
		}

		bool *previous_valid = packet.type == LINK_TYPE_KEYBOARD ?
			&previous_keyboard_valid : &previous_consumer_valid;
		bool *previous_queue_failed = packet.type == LINK_TYPE_KEYBOARD ?
			&previous_keyboard_queue_failed : &previous_consumer_queue_failed;
		uint8_t *previous_data = packet.type == LINK_TYPE_KEYBOARD ?
			previous_keyboard : previous_consumer;
		uint8_t *previous_sequence = packet.type == LINK_TYPE_KEYBOARD ?
			&previous_keyboard_sequence : &previous_consumer_sequence;
		bool const sequence_newer = !*previous_valid ||
			sequence_is_newer(packet.sequence, *previous_sequence);
		bool const same_sequence_retry = *previous_valid &&
			packet.sequence == *previous_sequence && *previous_queue_failed;

		/*
		 * Keyboard packets are absolute states: if the data is identical to
		 * the current keyboard state, it is a keepalive/duplicate, so drop it
		 * and do NOT re-queue it to Windows.
		 */
		if (*previous_valid && !same_sequence_retry &&
		    packet.type == LINK_TYPE_KEYBOARD &&
		    memcmp(packet.data, previous_data, INPUT_DATA_SIZE) == 0) {
			*previous_sequence = packet.sequence;
			*previous_queue_failed = false;
			atomic_inc(&radio_duplicates);
			continue;
		}

		if (packet.type == LINK_TYPE_KEYBOARD) {
			atomic_inc(&keyboard_rx_generation);
		}

		if (*previous_valid && !sequence_newer && !same_sequence_retry) {
			atomic_inc(&radio_duplicates);
			continue;
		}

		if (queue_hid_report(&packet)) {
			*previous_valid = true;
			*previous_sequence = packet.sequence;
			*previous_queue_failed = false;
			memcpy(previous_data, packet.data, INPUT_DATA_SIZE);
		} else {
			*previous_valid = true;
			*previous_sequence = packet.sequence;
			*previous_queue_failed = true;
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

static uint8_t hid_tx_buffer[1U + INPUT_DATA_SIZE];
static uint8_t battery_feature_report[HID_FEATURE_REPORT_LEN];

static void battery_feature_build(uint8_t report[HID_FEATURE_REPORT_LEN])
{
	uint8_t *const payload = &report[1];
	k_spinlock_key_t key = k_spin_lock(&battery_cache_lock);
	uint32_t const age_ms = battery_cache_valid ?
		(k_uptime_get_32() - battery_last_update_ms) : UINT32_MAX;
	uint32_t const age_seconds = age_ms == UINT32_MAX ? 0xFFFFU :
		MIN(age_ms / 1000U, 0xFFFFU);

	report[0] = HID_REPORT_ID_BATTERY;
	payload[0] = battery_percentage;
	payload[1] = battery_state;
	payload[2] = (uint8_t)battery_millivolts;
	payload[3] = (uint8_t)(battery_millivolts >> 8);
	payload[4] = battery_sequence;
	payload[5] = battery_cache_valid ? battery_flags : 0U;
	payload[6] = (uint8_t)age_seconds;
	payload[7] = (uint8_t)(age_seconds >> 8);
	k_spin_unlock(&battery_cache_lock, key);
}

static int hid_get_report(const struct device *dev,
				  struct usb_setup_packet *setup,
				  int32_t *len, uint8_t **data)
{
	ARG_UNUSED(dev);

	uint8_t const report_type = setup->wValue >> 8;
	uint8_t const report_id = setup->wValue & 0xFFU;

	if (report_type == 3U && report_id == HID_REPORT_ID_BATTERY) {
		battery_feature_build(battery_feature_report);
		*data = battery_feature_report;
		*len = sizeof(battery_feature_report);
		return 0;
	}

	if (report_type == 3U && report_id == HID_REPORT_ID_DFU) {
		k_spinlock_key_t key = k_spin_lock(&dfu_state_lock);
		dfu_feature_report[0] = HID_REPORT_ID_DFU;
		dfu_feature_report[1] = dfu_current_status;
		dfu_feature_report[2] = dfu_progress_percent;
		dfu_feature_report[3] = (uint8_t)dfu_last_offset_div_4;
		dfu_feature_report[4] = (uint8_t)(dfu_last_offset_div_4 >> 8);
		dfu_feature_report[5] = 0;
		dfu_feature_report[6] = 0;
		dfu_feature_report[7] = 0;
		dfu_feature_report[8] = 0;
		k_spin_unlock(&dfu_state_lock, key);
		*data = dfu_feature_report;
		*len = sizeof(dfu_feature_report);
		return 0;
	}

	return -ENOTSUP;
}

static int hid_set_report(const struct device *dev,
				  struct usb_setup_packet *setup,
				  int32_t *len, uint8_t **data)
{
	ARG_UNUSED(dev);

	uint8_t const report_type = setup->wValue >> 8;
	uint8_t const report_id = setup->wValue & 0xFFU;

	if (report_type == 2U && report_id == HID_REPORT_ID_KEYBOARD &&
	    *len >= 1 && *data != NULL) {
		uint8_t const *const report = *data;
		if (*len >= 2 && report[0] == HID_REPORT_ID_KEYBOARD) {
			receiver_set_led_state(report[1]);
		} else if (*len == 1) {
			receiver_set_led_state(report[0]);
		} else {
			return -EINVAL;
		}
		return 0;
	}

	if (report_type == 3U && report_id == HID_REPORT_ID_DFU &&
	    *len >= 1 && *data != NULL) {
		uint8_t const *const report = *data;
		uint8_t const *const payload = (*len >= DFU_FEATURE_REPORT_LEN && report[0] == HID_REPORT_ID_DFU) ?
			&report[1] : report;
		k_spinlock_key_t key = k_spin_lock(&dfu_state_lock);
		dfu_pending_ack.magic = LINK_ACK_MAGIC;
		dfu_pending_ack.version = LINK_VERSION;
		dfu_pending_ack.type = LINK_ACK_TYPE_DFU;
		dfu_pending_ack.sequence++;
		memcpy(dfu_pending_ack.data, payload, INPUT_DATA_SIZE);
		dfu_ack_active = true;
		dfu_current_status = DFU_STATUS_BUSY;
		k_spin_unlock(&dfu_state_lock, key);
		atomic_set(&led_ack_pending, 1);
		return 0;
	}

	if (report_type == 3U && report_id == HID_REPORT_ID_BATTERY) {
		/* Refresh now: attach one request bit to the next bounded ACK payload.
		 * There is no retry loop; FIFO-full waits for the next ESB RX event. */
		atomic_set(&battery_poll_requested, 1);
		atomic_set(&led_ack_dirty, 1);
		atomic_set(&led_ack_pending, 1);
		return 0;
	}

	return -ENOTSUP;
}

static void hid_int_in_ready(const struct device *dev)
{
	ARG_UNUSED(dev);
	k_sem_give(&hid_in_idle);
}

static void hid_int_out_ready(const struct device *dev)
{
	uint8_t report[INPUT_DATA_SIZE] = { 0 };
	uint32_t report_len = 0;

	if (hid_int_ep_read(dev, report, sizeof(report), &report_len) != 0 ||
	    report_len == 0) {
		return;
	}

	/* Report protocol carries ID 1; boot protocol sends the one-byte LED
	 * payload without a report ID. Use the length to disambiguate LED bit 0x01
	 * from report ID 1. */
	uint32_t const offset = report_len >= 2U &&
		report[0] == HID_REPORT_ID_KEYBOARD ? 1U : 0U;
	if (report_len > offset) {
		receiver_set_led_state(report[offset]);
	}
}

static const struct hid_ops hid_ops = {
	.get_report = hid_get_report,
	.set_report = hid_set_report,
	.int_in_ready = hid_int_in_ready,
	.int_out_ready = hid_int_out_ready,
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
		k_sem_reset(&hid_in_idle);
		k_sem_give(&hid_in_idle);
		break;
	case USB_DC_RESUME:
		atomic_set(&usb_suspended, 0);
		break;
	case USB_DC_RESET:
	case USB_DC_DISCONNECTED:
		atomic_set(&usb_configured, 0);
		atomic_set(&usb_suspended, 0);
		/* Release a sender if the endpoint disappeared. */
		k_sem_reset(&hid_in_idle);
		k_sem_give(&hid_in_idle);
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
	if (!usb_hid_ready()) {
		return -ENOTCONN;
	}
	if (report_size > sizeof(hid_tx_buffer)) {
		return -EMSGSIZE;
	}
	if (k_sem_take(&hid_in_idle, K_NO_WAIT) != 0) {
		return -EBUSY;
	}

	memcpy(hid_tx_buffer, report, report_size);
	int const err = hid_int_ep_write(hid_device, hid_tx_buffer,
					 report_size, NULL);
	if (err != 0) {
		k_sem_give(&hid_in_idle);
		atomic_inc(&hid_write_errors);
		return err;
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

int main(void)
{
	struct link_input_packet hid_pending = { 0 };
	bool hid_pending_valid = false;
	bool hid_pending_is_release_repeat = false;
	bool release_repeat_pending = false;
	uint32_t release_repeat_after_ms = 0;
	uint32_t release_repeat_generation = 0;
	int err;

	LOG_INF("Starting ESB-to-USB keyboard and consumer-control receiver");
	windows_led_epoch = (uint8_t)k_cycle_get_32();

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
		uint32_t const now = k_uptime_get_32();
		receiver_ack_task();

		if (!usb_hid_ready()) {
			/*
			 * Retain only keyboard state while the PC is absent. Consumer
			 * actions are transitions and must not be replayed later.
			 */
			while (k_msgq_get(&hid_report_queue, &queued,
					  K_NO_WAIT) == 0) {
				/* Input transitions are not replayed after USB reconnect. */
			}
			hid_pending_valid = false;
			release_repeat_pending = false;
			k_msleep(1);
			continue;
		}

		if (!hid_pending_valid &&
		    k_msgq_get(&hid_report_queue, &queued, K_NO_WAIT) == 0) {
			hid_pending = queued;
			hid_pending_valid = true;
			hid_pending_is_release_repeat = false;
			if (queued.type == LINK_TYPE_KEYBOARD &&
			    memcmp(queued.data, (uint8_t[INPUT_DATA_SIZE]){ 0 },
				   INPUT_DATA_SIZE) != 0) {
				release_repeat_pending = false;
			}
		}

		if (!hid_pending_valid && release_repeat_pending &&
		    (int32_t)(now - release_repeat_after_ms) >= 0) {
			if ((uint32_t)atomic_get(&keyboard_rx_generation) ==
			    release_repeat_generation) {
				hid_pending.type = LINK_TYPE_KEYBOARD;
				memset(hid_pending.data, 0, sizeof(hid_pending.data));
				hid_pending_valid = true;
				hid_pending_is_release_repeat = true;
			}
			release_repeat_pending = false;
		}

		if (hid_pending_valid) {
			int const send_result = hid_pending.type == LINK_TYPE_KEYBOARD ?
				send_keyboard_report(hid_pending.data) :
				send_consumer_report(hid_pending.data);

			if (send_result == 0) {
				if (hid_pending.type == LINK_TYPE_KEYBOARD) {
					if (memcmp(hid_pending.data,
						   (uint8_t[INPUT_DATA_SIZE]){ 0 },
						   INPUT_DATA_SIZE) == 0 &&
					    !hid_pending_is_release_repeat) {
						release_repeat_pending = true;
						release_repeat_after_ms = now +
							RELEASE_REPEAT_DELAY_MS;
						release_repeat_generation =
							(uint32_t)atomic_get(
								&keyboard_rx_generation);
					}
				}
				hid_pending_valid = false;
			}
		}

		/* No blocking HID wait: let the 1 ms USB endpoint callback release the
		 * idle semaphore while the radio handler remains free to queue input. */
		k_yield();
	}
}
