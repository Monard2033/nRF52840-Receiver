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
#define LINK_CONTROL_SESSION_RESET 0x03U
#define LINK_ACK_MAGIC         0x5AU
#define LINK_ACK_TYPE_LOCK_STATE 0x01U
#define LINK_RF_CHANNEL        90U
#define HID_QUEUE_DEPTH        256U
#define HID_REPORT_ID_KEYBOARD 0x01U
#define HID_REPORT_ID_CONSUMER 0x02U
#define HID_REPORT_ID_BATTERY  0x03U
#define HID_REPORT_ID_DFU      0x04U
#define HID_REPORT_ID_TRACE    0x05U
#define HID_FEATURE_PAYLOAD_LEN 10U
#define HID_FEATURE_REPORT_LEN  (1U + HID_FEATURE_PAYLOAD_LEN)

/* Battery flags bits (payload[5]). */
#define BATTERY_FLAG_VALID        0x01U
#define BATTERY_FLAG_MATERIAL     0x02U
#define BATTERY_FLAG_ETA_VALID    0x04U
#define BATTERY_FLAG_MASK         0x07U
#define BATTERY_ETA_UNKNOWN       0xFFFFU

/*
 * Battery discharge ETA estimator.
 *
 * The Receiver is the only node that observes the battery continuously, so it
 * keeps the discharge history itself and hands the tray a ready-made number.
 * A tray restart therefore shows the countdown immediately instead of waiting
 * for a fresh warm-up window.
 *
 * Voltage is a state, not a flow: a load step (keypress, radio burst, LED)
 * sags the cell and recovers, which a raw first/last difference would read as a
 * huge fake discharge rate. The slope is therefore measured between the median
 * of a block of the oldest samples and the median of a block of the newest
 * samples, over the time between the two block centres.
 */
#define BATT_ETA_HISTORY_LEN      1024U   /* ~17 min at 1 Hz */
#define BATT_ETA_MEDIAN_LEN       5U      /* 5 s median filter on raw mV */
#define BATT_ETA_BLOCK_LEN        4U      /* 4 s median per window endpoint */
#define BATT_ETA_RATE_SMOOTH_LEN  9U
#define BATT_ETA_MAX_WINDOW_MS    (60UL * 60UL * 1000UL)
#define BATT_ETA_MIN_WINDOW_MS    (5UL * 60UL * 1000UL)
#define BATT_ETA_GAP_RESET_MS     (6UL * 60UL * 60UL * 1000UL)
#define BATT_ETA_RISE_RESET_MV    25
#define BATT_ETA_MIN_DROP_MV      5U
#define BATT_ETA_MIN_MV           3050U   /* mirrors the RP2040 BATT_MIN_MV */
#define BATT_ETA_MAX_MV           4190U   /* mirrors the RP2040 BATT_MAX_MV */
#define BATT_ETA_MV_PER_PCT_X100  1140L   /* (4190-3050)/100 * 100 */
#define BATT_ETA_SAMPLE_MS        1000U
#define BATT_ETA_FRESH_TIMEOUT_MS 60000U  /* Max telemetry age before treating as asleep (packets sent every 20s) */
#define DFU_FEATURE_PAYLOAD_LEN 8U
#define DFU_FEATURE_REPORT_LEN  (1U + DFU_FEATURE_PAYLOAD_LEN)
#define TRACE_FEATURE_PAYLOAD_LEN 20U
#define TRACE_FEATURE_REPORT_LEN  (1U + TRACE_FEATURE_PAYLOAD_LEN)
#define TRACE_QUEUE_DEPTH       256U
#define TRACE_CMD_CLEAR         0x01U
#define TRACE_CMD_FREEZE        0x02U
#define TRACE_CMD_RESUME        0x03U
#define BATTERY_VALID_MIN_MV    2800U
#define BATTERY_VALID_MAX_MV    4300U

#define LINK_TYPE_DFU_START     0x10U
#define LINK_TYPE_DFU_DATA      0x11U
#define LINK_TYPE_DFU_FINISH    0x12U
#define LINK_TYPE_DFU_STATUS    0x13U
#define LINK_TYPE_DFU_FIRST     0x10U
#define LINK_TYPE_DFU_LAST      0x1FU
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

enum trace_stage {
	TRACE_STAGE_NONE = 0,
	TRACE_STAGE_ESB_RX = 1,
	TRACE_STAGE_HID_QUEUED = 2,
	TRACE_STAGE_HID_QUEUE_OVERFLOW = 3,
	TRACE_STAGE_DUPLICATE_DROP = 4,
	TRACE_STAGE_SEQUENCE_DROP = 5,
	TRACE_STAGE_USB_NOT_READY_DROP = 6,
	TRACE_STAGE_HID_SUBMIT_OK = 7,
	TRACE_STAGE_HID_SUBMIT_BUSY = 8,
	TRACE_STAGE_HID_SUBMIT_ERROR = 9,
	TRACE_STAGE_HID_COMPLETE = 10,
};

struct trace_entry {
	uint32_t timestamp_ms;
	uint8_t stage;
	uint8_t packet_type;
	uint8_t sequence;
	int8_t result;
	uint8_t data[INPUT_DATA_SIZE];
} __packed;

BUILD_ASSERT(sizeof(struct trace_entry) == 16U);

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

	/* Diagnostic-only, read-on-demand trace record (ID 5). No reports are
	 * emitted autonomously, so this cannot add traffic to the input path. */
	0x06, 0x02, 0xFF,
	0x09, 0x01,
	0xA1, 0x01,
	0x85, HID_REPORT_ID_TRACE,
	0x15, 0x00,
	0x26, 0xFF, 0x00,
	0x75, 0x08,
	0x95, TRACE_FEATURE_PAYLOAD_LEN,
	0x19, 0x01,
	0x29, TRACE_FEATURE_PAYLOAD_LEN,
	0xB1, 0x02,
	0xC0,
};
static const struct device *hid_device;
static atomic_t usb_configured;
static atomic_t usb_suspended;
static atomic_t last_rx_uptime_ms;

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

static struct k_spinlock trace_lock;
static struct trace_entry trace_queue[TRACE_QUEUE_DEPTH];
static uint16_t trace_head;
static uint16_t trace_tail;
static uint16_t trace_count;
static uint16_t trace_overwrites;
static bool trace_frozen;
static struct link_input_packet trace_inflight;
static bool trace_inflight_valid;

static struct k_spinlock battery_cache_lock;
static uint8_t battery_percentage;
static uint8_t battery_state;
static uint16_t battery_millivolts;
static uint8_t battery_sequence;
static uint8_t battery_flags;
static uint32_t battery_last_update_ms;
static bool battery_cache_valid;
static uint16_t battery_eta_minutes;
static bool battery_eta_valid;

/* Discharge ETA estimator state (only touched by the ETA thread). */
struct batt_eta_sample {
	uint32_t time_ms;
	uint16_t millivolts;
};

static struct batt_eta_sample batt_eta_history[BATT_ETA_HISTORY_LEN];
static uint16_t batt_eta_count;
static uint16_t batt_eta_head;
static uint16_t batt_eta_med[BATT_ETA_MEDIAN_LEN];
static uint16_t batt_eta_med_count;
static uint16_t batt_eta_med_head;
static int32_t batt_eta_rate[BATT_ETA_RATE_SMOOTH_LEN];
static uint16_t batt_eta_rate_count;
static uint16_t batt_eta_rate_head;
static uint32_t batt_eta_last_ms;
static bool batt_eta_has_last;

static uint16_t batt_eta_median_u16(uint16_t *values, uint16_t n)
{
	for (uint16_t i = 1U; i < n; ++i) {
		uint16_t const key = values[i];
		uint16_t j = i;

		while (j > 0U && values[j - 1U] > key) {
			values[j] = values[j - 1U];
			--j;
		}
		values[j] = key;
	}
	return values[n / 2U];
}

static uint16_t batt_eta_median_filter_value(void)
{
	uint16_t scratch[BATT_ETA_MEDIAN_LEN];

	for (uint16_t i = 0U; i < batt_eta_med_count; ++i) {
		scratch[i] = batt_eta_med[i];
	}
	return batt_eta_median_u16(scratch, batt_eta_med_count);
}

static uint16_t batt_eta_block_median(uint16_t from, uint16_t block_len)
{
	uint16_t scratch[BATT_ETA_BLOCK_LEN];

	for (uint16_t i = 0U; i < block_len; ++i) {
		scratch[i] = batt_eta_history[(from + i) % BATT_ETA_HISTORY_LEN]
				     .millivolts;
	}
	return batt_eta_median_u16(scratch, block_len);
}

static void batt_eta_reset(void)
{
	batt_eta_count = 0U;
	batt_eta_head = 0U;
	batt_eta_med_count = 0U;
	batt_eta_med_head = 0U;
	batt_eta_rate_count = 0U;
	batt_eta_rate_head = 0U;
	batt_eta_has_last = false;
}

static int32_t batt_eta_median_rate(void)
{
	int32_t sorted[BATT_ETA_RATE_SMOOTH_LEN];

	if (batt_eta_rate_count == 0U) {
		return 0;
	}
	for (uint16_t i = 0U; i < batt_eta_rate_count; ++i) {
		sorted[i] = batt_eta_rate[i];
	}
	for (uint16_t i = 1U; i < batt_eta_rate_count; ++i) {
		int32_t const key = sorted[i];
		uint16_t j = i;

		while (j > 0U && sorted[j - 1U] > key) {
			sorted[j] = sorted[j - 1U];
			--j;
		}
		sorted[j] = key;
	}
	return sorted[batt_eta_rate_count / 2U];
}

/*
 * Fit the discharge slope over the newest window. Returns true and fills
 * rate_x100 (mV per minute, scaled by 100) when a usable slope exists.
 */
static bool batt_eta_fit_rate(int32_t *rate_x100)
{
	if (batt_eta_count < (BATT_ETA_BLOCK_LEN * 2U)) {
		return false;
	}

	uint32_t const newest_time =
		batt_eta_history[(batt_eta_head + BATT_ETA_HISTORY_LEN - 1U) %
				 BATT_ETA_HISTORY_LEN].time_ms;

	/* Index (from the oldest kept sample) where the window starts. */
	uint16_t index = 0U;

	while (index + 1U < batt_eta_count) {
		uint16_t const slot =
			(batt_eta_head + BATT_ETA_HISTORY_LEN - batt_eta_count + index) %
			BATT_ETA_HISTORY_LEN;

		if ((newest_time - batt_eta_history[slot].time_ms) <=
		    BATT_ETA_MAX_WINDOW_MS) {
			break;
		}
		++index;
	}

	uint16_t const span = batt_eta_count - index;
	uint16_t block_len = span / 2U;

	if (block_len > BATT_ETA_BLOCK_LEN) {
		block_len = BATT_ETA_BLOCK_LEN;
	}
	if (block_len == 0U) {
		return false;
	}

	uint16_t const oldest_block = index;
	uint16_t const newest_block = batt_eta_count - block_len;

	int32_t const oldest_mv = (int32_t)batt_eta_block_median(oldest_block,
							block_len);
	int32_t const newest_mv = (int32_t)batt_eta_block_median(newest_block,
							block_len);

	uint16_t const oldest_centre =
		(batt_eta_head + BATT_ETA_HISTORY_LEN - batt_eta_count +
		 oldest_block + (block_len / 2U)) % BATT_ETA_HISTORY_LEN;
	uint16_t const newest_centre =
		(batt_eta_head + BATT_ETA_HISTORY_LEN - batt_eta_count +
		 newest_block + (block_len / 2U)) % BATT_ETA_HISTORY_LEN;

	uint32_t const t_old = batt_eta_history[oldest_centre].time_ms;
	uint32_t const t_new = batt_eta_history[newest_centre].time_ms;

	if (t_new <= t_old) {
		return false;
	}

	uint32_t const elapsed_ms = t_new - t_old;

	if (elapsed_ms < BATT_ETA_MIN_WINDOW_MS) {
		return false;
	}

	int32_t const drop_mv = oldest_mv - newest_mv;

	if (drop_mv < (int32_t)BATT_ETA_MIN_DROP_MV) {
		return false;
	}

	*rate_x100 = (int32_t)(((int64_t)drop_mv * 6000000LL) /
			       (int64_t)elapsed_ms);
	return *rate_x100 > 0;
}

static void batt_eta_update(uint32_t now_ms, uint16_t millivolts)
{
	if (batt_eta_has_last &&
	    (now_ms - batt_eta_last_ms) > BATT_ETA_GAP_RESET_MS) {
		batt_eta_reset();
	}

	/*
	 * A rise this large means a charge cycle happened while we were not
	 * looking, so the stored history is no longer one continuous discharge.
	 */
	if (batt_eta_med_count > 0U &&
	    ((int32_t)millivolts -
	     (int32_t)batt_eta_median_filter_value()) > BATT_ETA_RISE_RESET_MV) {
		batt_eta_reset();
	}

	batt_eta_history[batt_eta_head].time_ms = now_ms;
	batt_eta_history[batt_eta_head].millivolts = millivolts;
	batt_eta_head = (uint16_t)((batt_eta_head + 1U) % BATT_ETA_HISTORY_LEN);
	if (batt_eta_count < BATT_ETA_HISTORY_LEN) {
		batt_eta_count++;
	}
	batt_eta_last_ms = now_ms;
	batt_eta_has_last = true;

	batt_eta_med[batt_eta_med_head] = millivolts;
	batt_eta_med_head =
		(uint16_t)((batt_eta_med_head + 1U) % BATT_ETA_MEDIAN_LEN);
	if (batt_eta_med_count < BATT_ETA_MEDIAN_LEN) {
		batt_eta_med_count++;
	}

	int32_t rate_x100 = 0;
	bool valid = false;
	uint16_t minutes = BATTERY_ETA_UNKNOWN;

	if (batt_eta_fit_rate(&rate_x100)) {
		batt_eta_rate[batt_eta_rate_head] = rate_x100;
		batt_eta_rate_head =
			(uint16_t)((batt_eta_rate_head + 1U) %
				   BATT_ETA_RATE_SMOOTH_LEN);
		if (batt_eta_rate_count < BATT_ETA_RATE_SMOOTH_LEN) {
			batt_eta_rate_count++;
		}

		int32_t const smoothed = batt_eta_median_rate();

		if (smoothed > 0) {
			int32_t const remaining_mv =
				(int32_t)batt_eta_median_filter_value() -
				(int32_t)BATT_ETA_MIN_MV;

			if (remaining_mv > 0) {
				int64_t const mins =
					((int64_t)remaining_mv * 100LL) /
					(int64_t)smoothed;

				minutes = (uint16_t)MIN(mins,
							(int64_t)BATTERY_ETA_UNKNOWN - 1);
				valid = true;
			}
		}
	}

	k_spinlock_key_t key = k_spin_lock(&battery_cache_lock);
	battery_eta_minutes = minutes;
	battery_eta_valid = valid;
	k_spin_unlock(&battery_cache_lock, key);
}

K_MSGQ_DEFINE(dfu_ack_queue, sizeof(struct link_ack_frame), 64, 4);
static struct k_spinlock dfu_state_lock;
static struct link_ack_frame dfu_pending_ack;
static bool dfu_ack_active;
static uint8_t dfu_current_status = DFU_STATUS_IDLE;
static uint8_t dfu_status_session;
static uint8_t dfu_status_token;
static uint8_t dfu_status_detail;
static uint32_t dfu_status_value;
static uint8_t dfu_command_sequence;
static uint8_t dfu_feature_report[DFU_FEATURE_REPORT_LEN];

static void trace_record(uint8_t stage,
			 const struct link_input_packet *packet, int result)
{
	struct trace_entry entry = {
		.timestamp_ms = k_uptime_get_32(),
		.stage = stage,
		.packet_type = packet != NULL ? packet->type : 0U,
		.sequence = packet != NULL ? packet->sequence : 0U,
		.result = (int8_t)CLAMP(result, INT8_MIN, INT8_MAX),
	};

	if (packet != NULL) {
		memcpy(entry.data, packet->data, sizeof(entry.data));
	}

	k_spinlock_key_t key = k_spin_lock(&trace_lock);
	if (!trace_frozen) {
		if (trace_count == TRACE_QUEUE_DEPTH) {
			trace_tail = (uint16_t)((trace_tail + 1U) % TRACE_QUEUE_DEPTH);
			trace_count--;
			trace_overwrites++;
		}
		trace_queue[trace_head] = entry;
		trace_head = (uint16_t)((trace_head + 1U) % TRACE_QUEUE_DEPTH);
		trace_count++;
	}
	k_spin_unlock(&trace_lock, key);
}

static void trace_inflight_prepare(const struct link_input_packet *packet)
{
	k_spinlock_key_t key = k_spin_lock(&trace_lock);
	trace_inflight = *packet;
	trace_inflight_valid = true;
	k_spin_unlock(&trace_lock, key);
}

static void trace_inflight_cancel(void)
{
	k_spinlock_key_t key = k_spin_lock(&trace_lock);
	trace_inflight_valid = false;
	k_spin_unlock(&trace_lock, key);
}

static void trace_record_inflight_complete(void)
{
	struct link_input_packet packet;
	bool valid;
	k_spinlock_key_t key = k_spin_lock(&trace_lock);
	valid = trace_inflight_valid;
	if (valid) {
		packet = trace_inflight;
		trace_inflight_valid = false;
	}
	k_spin_unlock(&trace_lock, key);

	if (valid) {
		trace_record(TRACE_STAGE_HID_COMPLETE, &packet, 0);
	}
}

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
	trace_record(TRACE_STAGE_HID_QUEUE_OVERFLOW, packet, -ENOSPC);
	if (k_msgq_get(&hid_report_queue, &discarded, K_NO_WAIT) == 0) {
		return k_msgq_put(&hid_report_queue, packet, K_NO_WAIT) == 0;
	}
	return false;
}

/* DFU OTA command range (0x10..0x1F). DFU frames are strictly ordered,
 * opaque protocol data; every suppression path must bypass them. */
static bool frame_is_dfu_command(uint8_t type)
{
	return type >= LINK_TYPE_DFU_FIRST && type <= LINK_TYPE_DFU_LAST;
}

static bool battery_packet_is_valid(const struct link_input_packet *packet)
{
	uint16_t const millivolts = (uint16_t)packet->data[2] |
		((uint16_t)packet->data[3] << 8);

	return packet->data[0] <= 100U && packet->data[1] <= 4U &&
	       millivolts >= BATTERY_VALID_MIN_MV &&
	       millivolts <= BATTERY_VALID_MAX_MV &&
	       (packet->data[5] & 0x01U) != 0U &&
	       (packet->data[5] & 0xF0U) == 0U &&
	       packet->data[6] == 0U && packet->data[7] == 0U;
}

static void battery_cache_update(const struct link_input_packet *packet)
{
	k_spinlock_key_t key = k_spin_lock(&battery_cache_lock);
	battery_percentage = packet->data[0];
	battery_state = packet->data[1];
	battery_millivolts = (uint16_t)packet->data[2] |
		((uint16_t)packet->data[3] << 8);
	battery_sequence = packet->data[4];
	battery_flags = packet->data[5];
	battery_last_update_ms = k_uptime_get_32();
	battery_cache_valid = true;
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

static bool receiver_queue_dfu_ack(void)
{
	k_spinlock_key_t key = k_spin_lock(&dfu_state_lock);
	bool queued = false;

	for (;;) {
		if (!dfu_ack_active) {
			struct link_ack_frame next;
			if (k_msgq_get(&dfu_ack_queue, &next, K_NO_WAIT) != 0) {
				break;
			}
			dfu_pending_ack = next;
			dfu_ack_active = true;
		}

		struct esb_payload payload = { 0 };
		payload.length = sizeof(dfu_pending_ack);
		payload.pipe = 0;
		payload.noack = false;
		memcpy(payload.data, &dfu_pending_ack, sizeof(dfu_pending_ack));

		int err = esb_write_payload(&payload);
		if (err != 0) {
			break;
		}

		dfu_ack_active = false;
		queued = true;
	}

	k_spin_unlock(&dfu_state_lock, key);
	return queued;
}

static void receiver_ack_task(void)
{
	(void)receiver_queue_dfu_ack();

	if (atomic_cas(&led_ack_pending, 1, 0)) {
		k_spinlock_key_t key = k_spin_lock(&dfu_state_lock);
		bool const dfu_idle = !dfu_ack_active &&
			(k_msgq_num_used_get(&dfu_ack_queue) == 0);
		k_spin_unlock(&dfu_state_lock, key);

		if (dfu_idle) {
			struct link_ack_frame ack = {
				.magic = LINK_ACK_MAGIC,
				.version = LINK_VERSION,
				.type = 0x00U,
			};
			k_spinlock_key_t led_key = k_spin_lock(&led_state_lock);
			if (windows_led_valid) {
				ack.type = LINK_ACK_TYPE_LOCK_STATE;
				ack.sequence = windows_led_sequence;
				ack.data[0] = windows_led_state;
				ack.data[1] = 0x01U;
				ack.data[2] = windows_led_epoch;
			}
			k_spin_unlock(&led_state_lock, led_key);

			struct esb_payload payload = { 0 };
			payload.length = sizeof(ack);
			payload.pipe = 0;
			payload.noack = false;
			memcpy(payload.data, &ack, sizeof(ack));
			(void)esb_write_payload(&payload);
		}
	}
}

static void receiver_reset_input_session(void)
{
	bool const keyboard_was_pressed = previous_keyboard_valid &&
		memcmp(previous_keyboard, (uint8_t[INPUT_DATA_SIZE]){ 0 },
		       INPUT_DATA_SIZE) != 0;
	bool const consumer_was_pressed = previous_consumer_valid &&
		memcmp(previous_consumer, (uint8_t[INPUT_DATA_SIZE]){ 0 },
		       INPUT_DATA_SIZE) != 0;

	previous_keyboard_valid = false;
	previous_consumer_valid = false;
	previous_keyboard_queue_failed = false;
	previous_consumer_queue_failed = false;
	previous_keyboard_sequence = 0U;
	previous_consumer_sequence = 0U;
	memset(previous_keyboard, 0, sizeof(previous_keyboard));
	memset(previous_consumer, 0, sizeof(previous_consumer));

	k_spinlock_key_t key = k_spin_lock(&battery_cache_lock);
	battery_cache_valid = false;
	battery_flags = 0U;
	battery_sequence = 0U;
	battery_last_update_ms = 0U;
	k_spin_unlock(&battery_cache_lock, key);

	/* Release only a state that was actually held in the previous RP2040
	 * session. The first new real frame is then accepted regardless of seq. */
	if (keyboard_was_pressed) {
		struct link_input_packet const release = {
			.magic = LINK_MAGIC,
			.version = LINK_VERSION,
			.type = LINK_TYPE_KEYBOARD,
		};
		(void)queue_hid_report(&release);
	}
	if (consumer_was_pressed) {
		struct link_input_packet const release = {
			.magic = LINK_MAGIC,
			.version = LINK_VERSION,
			.type = LINK_TYPE_CONSUMER,
		};
		(void)queue_hid_report(&release);
	}
}

static void receiver_esb_event_handler(const struct esb_evt *event)
{
	if (event->evt_id == ESB_EVENT_TX_SUCCESS) {
		(void)receiver_queue_dfu_ack();
		return;
	}

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
		atomic_set(&last_rx_uptime_ms, (long)k_uptime_get_32());

		if (packet.magic != LINK_MAGIC || packet.version != LINK_VERSION) {
			atomic_inc(&radio_bad_packets);
			continue;
		}

		if (packet.type == LINK_TYPE_DFU_STATUS) {
			k_spinlock_key_t key = k_spin_lock(&dfu_state_lock);
			dfu_current_status = packet.data[0];
			dfu_status_session = packet.data[1];
			dfu_status_token = packet.data[2];
			dfu_status_detail = packet.data[3];
			dfu_status_value = (uint32_t)packet.data[4] |
				((uint32_t)packet.data[5] << 8) |
				((uint32_t)packet.data[6] << 16) |
				((uint32_t)packet.data[7] << 24);
			k_spin_unlock(&dfu_state_lock, key);
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
			if (packet.data[0] == LINK_CONTROL_SESSION_RESET) {
				receiver_reset_input_session();
				continue;
			}
			if (packet.data[0] != LINK_CONTROL_POLL_ACK) {
				atomic_inc(&radio_bad_packets);
				continue;
			}
			continue;
		}

		trace_record(TRACE_STAGE_ESB_RX, &packet, 0);

		bool *previous_valid = packet.type == LINK_TYPE_KEYBOARD ?
			&previous_keyboard_valid : &previous_consumer_valid;
		bool *previous_queue_failed = packet.type == LINK_TYPE_KEYBOARD ?
			&previous_keyboard_queue_failed : &previous_consumer_queue_failed;
		uint8_t *previous_data = packet.type == LINK_TYPE_KEYBOARD ?
			previous_keyboard : previous_consumer;
		uint8_t *previous_sequence = packet.type == LINK_TYPE_KEYBOARD ?
			&previous_keyboard_sequence : &previous_consumer_sequence;
		bool const same_sequence_retry = *previous_valid &&
			packet.sequence == *previous_sequence && *previous_queue_failed;
		bool const data_changed = !*previous_valid ||
			memcmp(packet.data, previous_data, INPUT_DATA_SIZE) != 0;

		/*
		 * If the payload is identical to the current active state and sequence
		 * has not changed, drop it as an ESB duplicate retransmission.
		 * DFU frames (0x10..0x1F) are never suppressed: the RP2040 DFU handler
		 * owns all duplicate/replay policy for the OTA stream. */
		if (!frame_is_dfu_command(packet.type) &&
		    !data_changed && packet.sequence == *previous_sequence && !same_sequence_retry) {
			atomic_inc(&radio_duplicates);
			trace_record(TRACE_STAGE_DUPLICATE_DROP, &packet, 0);
			continue;
		}

		/*
		 * Any valid state transition (key press, key release, combo, consumer action)
		 * is immediately accepted and delivered to Windows.
		 */
		if (queue_hid_report(&packet)) {
			trace_record(TRACE_STAGE_HID_QUEUED, &packet, 0);
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

	(void)receiver_queue_dfu_ack();
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
static uint8_t trace_feature_report[TRACE_FEATURE_REPORT_LEN];

static void battery_feature_build(uint8_t report[HID_FEATURE_REPORT_LEN])
{
	uint8_t *const payload = &report[1];
	k_spinlock_key_t key = k_spin_lock(&battery_cache_lock);
	uint32_t const age_ms = battery_cache_valid ?
		(k_uptime_get_32() - battery_last_update_ms) : UINT32_MAX;
	uint32_t const age_seconds = age_ms == UINT32_MAX ? 0xFFFFU :
		MIN(age_ms / 1000U, 0xFFFFU);
	uint16_t const eta_minutes = battery_eta_valid ? battery_eta_minutes :
		BATTERY_ETA_UNKNOWN;
	uint8_t const eta_bit = battery_eta_valid ? BATTERY_FLAG_ETA_VALID : 0U;

	report[0] = HID_REPORT_ID_BATTERY;
	payload[0] = battery_percentage;
	payload[1] = battery_state;
	payload[2] = (uint8_t)battery_millivolts;
	payload[3] = (uint8_t)(battery_millivolts >> 8);
	payload[4] = battery_sequence;
	payload[5] = (battery_cache_valid ? battery_flags : 0U) | eta_bit;
	payload[6] = (uint8_t)age_seconds;
	payload[7] = (uint8_t)(age_seconds >> 8);
	payload[8] = (uint8_t)eta_minutes;
	payload[9] = (uint8_t)(eta_minutes >> 8);
	k_spin_unlock(&battery_cache_lock, key);
}

static void trace_feature_build(uint8_t report[TRACE_FEATURE_REPORT_LEN])
{
	struct trace_entry entry = { 0 };
	bool valid = false;
	uint8_t remaining;
	uint8_t overwrites;
	bool frozen;
	k_spinlock_key_t key = k_spin_lock(&trace_lock);

	if (trace_count != 0U) {
		entry = trace_queue[trace_tail];
		trace_tail = (uint16_t)((trace_tail + 1U) % TRACE_QUEUE_DEPTH);
		trace_count--;
		valid = true;
	}
	remaining = (uint8_t)MIN(trace_count, UINT8_MAX);
	overwrites = (uint8_t)MIN(trace_overwrites, UINT8_MAX);
	frozen = trace_frozen;
	k_spin_unlock(&trace_lock, key);

	memset(report, 0, TRACE_FEATURE_REPORT_LEN);
	report[0] = HID_REPORT_ID_TRACE;
	report[1] = 1U; /* Trace protocol version. */
	report[2] = (valid ? 0x01U : 0U) | (frozen ? 0x02U : 0U);
	report[3] = remaining;
	report[4] = overwrites;
	if (valid) {
		memcpy(&report[5], &entry, sizeof(entry));
	}
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
		dfu_feature_report[2] = dfu_status_session;
		dfu_feature_report[3] = dfu_status_token;
		dfu_feature_report[4] = dfu_status_detail;
		dfu_feature_report[5] = (uint8_t)dfu_status_value;
		dfu_feature_report[6] = (uint8_t)(dfu_status_value >> 8);
		dfu_feature_report[7] = (uint8_t)(dfu_status_value >> 16);
		dfu_feature_report[8] = (uint8_t)(dfu_status_value >> 24);
		k_spin_unlock(&dfu_state_lock, key);
		*data = dfu_feature_report;
		*len = sizeof(dfu_feature_report);
		return 0;
	}

	if (report_type == 3U && report_id == HID_REPORT_ID_TRACE) {
		trace_feature_build(trace_feature_report);
		*data = trace_feature_report;
		*len = sizeof(trace_feature_report);
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
		uint8_t const *payload;
		if (*len >= DFU_FEATURE_REPORT_LEN &&
		    report[0] == HID_REPORT_ID_DFU) {
			payload = &report[1];
		} else if (*len >= DFU_FEATURE_PAYLOAD_LEN) {
			payload = report;
		} else {
			return -EINVAL;
		}

		struct link_ack_frame ack = {
			.magic = LINK_ACK_MAGIC,
			.version = LINK_VERSION,
			.type = LINK_ACK_TYPE_DFU,
		};

		k_spinlock_key_t key = k_spin_lock(&dfu_state_lock);
		if (payload[0] == LINK_TYPE_DFU_START) {
			k_msgq_purge(&dfu_ack_queue);
			dfu_ack_active = false;
			dfu_command_sequence = 0U;
		}

		ack.sequence = ++dfu_command_sequence;
		memcpy(ack.data, payload, INPUT_DATA_SIZE);

		if (k_msgq_put(&dfu_ack_queue, &ack, K_NO_WAIT) != 0) {
			k_spin_unlock(&dfu_state_lock, key);
			return -EBUSY;
		}

		if (payload[0] == LINK_TYPE_DFU_START) {
			dfu_current_status = DFU_STATUS_BUSY;
			dfu_status_session = payload[1];
			dfu_status_token = ack.sequence;
			dfu_status_detail = 0;
			dfu_status_value = 0;
		}
		k_spin_unlock(&dfu_state_lock, key);

		(void)receiver_queue_dfu_ack();
		return 0;
	}

	if (report_type == 3U && report_id == HID_REPORT_ID_BATTERY) {
		/* Cached telemetry only. Battery polling must not create reverse ACK
		 * traffic on the keyboard hot path. */
		return 0;
	}

	if (report_type == 3U && report_id == HID_REPORT_ID_TRACE &&
	    *len >= 1 && *data != NULL) {
		uint8_t const *const report = *data;
		uint8_t const command = (*len >= TRACE_FEATURE_REPORT_LEN &&
			report[0] == HID_REPORT_ID_TRACE) ? report[1] : report[0];
		k_spinlock_key_t key = k_spin_lock(&trace_lock);
		switch (command) {
		case TRACE_CMD_CLEAR:
			trace_head = 0U;
			trace_tail = 0U;
			trace_count = 0U;
			trace_overwrites = 0U;
			break;
		case TRACE_CMD_FREEZE:
			trace_frozen = true;
			break;
		case TRACE_CMD_RESUME:
			trace_frozen = false;
			break;
		default:
			k_spin_unlock(&trace_lock, key);
			return -EINVAL;
		}
		k_spin_unlock(&trace_lock, key);
		return 0;
	}

	return -ENOTSUP;
}

static void hid_int_in_ready(const struct device *dev)
{
	ARG_UNUSED(dev);
	trace_record_inflight_complete();
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

static int send_hid_report(const uint8_t *report, size_t report_size,
			   const struct link_input_packet *trace_packet)
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

	trace_inflight_prepare(trace_packet);
	memcpy(hid_tx_buffer, report, report_size);
	int const err = hid_int_ep_write(hid_device, hid_tx_buffer,
					 report_size, NULL);
	if (err != 0) {
		trace_inflight_cancel();
		k_sem_give(&hid_in_idle);
		atomic_inc(&hid_write_errors);
		return err;
	}

	trace_record(TRACE_STAGE_HID_SUBMIT_OK, trace_packet, 0);
	atomic_inc(&hid_reports_sent);
	return 0;
}

static int send_keyboard_report(const struct link_input_packet *packet)
{
	uint8_t report[1U + INPUT_DATA_SIZE];

	report[0] = HID_REPORT_ID_KEYBOARD;
	memcpy(&report[1], packet->data, INPUT_DATA_SIZE);
	return send_hid_report(report, sizeof(report), packet);
}

static int send_consumer_report(const struct link_input_packet *packet)
{
	const uint8_t report[] = {
		HID_REPORT_ID_CONSUMER,
		packet->data[0],
		packet->data[1],
	};

	return send_hid_report(report, sizeof(report), packet);
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

/*
 * Battery ETA sampling.
 *
 * NOTE: this is deliberately NOT a separate low-priority thread. main() runs
 * at priority 0 and never blocks (it only calls k_yield(), which in Zephyr
 * yields only to threads of the SAME priority), so any lower-priority thread
 * would be starved forever. The work is therefore rate-limited from the main
 * loop instead: it costs a few microseconds once per second.
 */
static void battery_eta_task(void)
{
	static uint32_t last_sample_ms;
	static uint8_t last_state = 0xFFU;
	static bool have_state;

	uint32_t const now = k_uptime_get_32();

	if (have_state && (now - last_sample_ms) < BATT_ETA_SAMPLE_MS) {
		return;
	}
	last_sample_ms = now;

	uint8_t pct;
	uint8_t state;
	uint16_t millivolts;
	uint32_t last_update_ms;
	bool valid;
	k_spinlock_key_t key = k_spin_lock(&battery_cache_lock);

	valid = battery_cache_valid;
	pct = battery_percentage;
	state = battery_state;
	millivolts = battery_millivolts;
	last_update_ms = battery_last_update_ms;
	k_spin_unlock(&battery_cache_lock, key);

	ARG_UNUSED(pct);

	if (valid && (state == 1U || state == 3U)) {
		/* Charging or full: a charge cycle voids the history. */
		if (!have_state || last_state != state) {
			batt_eta_reset();
			key = k_spin_lock(&battery_cache_lock);
			battery_eta_valid = false;
			battery_eta_minutes = BATTERY_ETA_UNKNOWN;
			k_spin_unlock(&battery_cache_lock, key);
		}
		have_state = true;
		last_state = state;
	} else if (valid && (state == 0U || state == 2U)) {
		if (have_state && (last_state == 1U || last_state == 3U)) {
			batt_eta_reset();
		}
		have_state = true;
		last_state = state;
		/*
		 * Only feed the discharge estimator while the keyboard is actively
		 * transmitting. When the keyboard goes to sleep, telemetry packets stop.
		 * Feeding identical millivolt readings every second would flatten the
		 * ring buffer and destroy the computed ETA after ~17 minutes.
		 * If telemetry is older than 60s, keep the existing computed ETA in cache.
		 */
		if ((now - last_update_ms) <= BATT_ETA_FRESH_TIMEOUT_MS) {
			batt_eta_update(now, millivolts);
		}
	} else {
		/* Unreported state: keep the history, do not feed it. */
		have_state = false;
		last_state = 0xFFU;
	}
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
	bool hid_pending_failure_logged = false;
	int err;

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
		receiver_ack_task();
		battery_eta_task();

		if (!usb_hid_ready()) {
			/*
			 * Retain only keyboard state while the PC is absent. Consumer
			 * actions are transitions and must not be replayed later.
			 */
			while (k_msgq_get(&hid_report_queue, &queued,
					  K_NO_WAIT) == 0) {
				trace_record(TRACE_STAGE_USB_NOT_READY_DROP, &queued,
					     -ENOTCONN);
			}
			hid_pending_valid = false;
			hid_pending_failure_logged = false;
			k_msleep(1);
			continue;
		}

		if (!hid_pending_valid &&
		    k_msgq_get(&hid_report_queue, &queued, K_NO_WAIT) == 0) {
			hid_pending = queued;
			hid_pending_valid = true;
			hid_pending_failure_logged = false;
		}

		if (hid_pending_valid) {
			int const send_result = hid_pending.type == LINK_TYPE_KEYBOARD ?
				send_keyboard_report(&hid_pending) :
				send_consumer_report(&hid_pending);

			if (send_result == 0) {
				hid_pending_valid = false;
				hid_pending_failure_logged = false;
			} else if (!hid_pending_failure_logged) {
				trace_record(send_result == -EBUSY ?
					TRACE_STAGE_HID_SUBMIT_BUSY :
					TRACE_STAGE_HID_SUBMIT_ERROR,
					&hid_pending, send_result);
				hid_pending_failure_logged = true;
			}
		}

		/* No blocking HID wait: let the 1 ms USB endpoint callback release the
		 * idle semaphore while the radio handler remains free to queue input. */
		k_yield();
	}
}
