/*
 * Copyright (c) 2024 Monard2033
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <esb.h>
#include <string.h>

#define REPORT_ID               1

LOG_MODULE_REGISTER(RECEIVER, LOG_LEVEL_INF);

/* --- ESB Configuration --- */
static struct esb_payload rx_payload;
static struct esb_payload tx_payload;
static struct esb_config esb_config = ESB_DEFAULT_CONFIG;
static uint8_t last_hid_report[8] = {0};
static bool new_report_available = false;

/* --- USB HID Configuration --- */
static const struct device *hid_dev;
static K_SEM_DEFINE(usb_configured_sem, 0, 1);
static volatile bool configured = false; /* Set by default for testing */

/* Key Configuration */
#define KEY_CTRL_CODE_MIN 224
#define KEY_CTRL_CODE_MAX 231
#define KEY_CODE_MIN      0
#define KEY_CODE_MAX      101
#define KEY_PRESS_MAX     6
#define INPUT_REPORT_KEYS_MAX_LEN (1 + 1 + KEY_PRESS_MAX)

BUILD_ASSERT((KEY_CTRL_CODE_MAX - KEY_CTRL_CODE_MIN) + 1 == 8);

/* Packet Types */
enum packet_type {
    PACKET_TYPE_GPIO_DATA = 0x05,
    PACKET_TYPE_BATTERY_VOLTAGE = 0x06
};

/* HID Report Descriptor for a standard keyboard */
static const uint8_t hid_report_desc[] = {
    0x05, 0x01, 0x09, 0x06, 0xA1, 0x01, 0x75, 0x01,
    0x95, 0x08, 0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7,
    0x15, 0x00, 0x25, 0x01, 0x81, 0x02, 0x95, 0x01,
    0x75, 0x08, 0x81, 0x01, 0x95, 0x06, 0x75, 0x08,
    0x15, 0x00, 0x25, 0x65, 0x05, 0x07, 0x19, 0x00,
    0x29, 0x65, 0x81, 0x00, 0xC0
};

/* Custom USB endpoint descriptor for 1ms polling */
static const uint8_t hid_ep_descriptor[] = {
    0x07,          // bLength: Endpoint descriptor size
    0x05,          // bDescriptorType: Endpoint
    0x81,          // bEndpointAddress: IN endpoint 1 (0x80 | 0x01)
    0x03,          // bmAttributes: Interrupt
    0x08, 0x00,    // wMaxPacketSize: 8 bytes (for HID report)
    0x01           // bInterval: 1ms polling interval
};

static void int_in_ready_cb(const struct device *dev) {
    if (configured && new_report_available) {
        LOG_INF("IN endpoint ready to send data. Sending report: %02x %02x %02x %02x %02x %02x %02x %02x",
                last_hid_report[0], last_hid_report[1], last_hid_report[2], last_hid_report[3],
                last_hid_report[4], last_hid_report[5], last_hid_report[6], last_hid_report[7]);
        if (hid_int_ep_write(dev, last_hid_report, 8, NULL)) {
            LOG_ERR("Failed to submit HID report");
        } else {
            new_report_available = false; // Clear flag after sending
        }
    }
}

static const struct hid_ops my_hid_ops = {
    .int_in_ready = int_in_ready_cb,
};

static void usb_status_cb(enum usb_dc_status_code status, const uint8_t *param) {
    ARG_UNUSED(param);
    LOG_DBG("USB status callback: status=%d", status);
    switch (status) {
    case USB_DC_CONFIGURED:
        LOG_INF("USB_DC_CONFIGURED received, setting configured = true");
        configured = true;
        k_sem_give(&usb_configured_sem);
        break;
    default:
        LOG_DBG("Unknown USB status code: %d", status);
        break;
    }
}

void receiver_esb_event_handler(struct esb_evt const *event) {
    switch (event->evt_id) {
    case ESB_EVENT_TX_SUCCESS:
        LOG_INF("TX success (ACK payload sent)");
        break;
    case ESB_EVENT_TX_FAILED:
        LOG_ERR("TX failed for ACK payload");
        break;
    case ESB_EVENT_RX_RECEIVED:
        while (esb_read_rx_payload(&rx_payload) == 0) {
            LOG_INF("Received Payload (length: %d bytes)", rx_payload.length);
            LOG_HEXDUMP_INF(rx_payload.data, rx_payload.length, "Raw Payload");
            if (rx_payload.length == 8) {
                uint8_t packet_type = rx_payload.data[0];
                if (packet_type == PACKET_TYPE_GPIO_DATA) {
                    LOG_INF("Received GPIO data: D+ = %d, D- = %d", rx_payload.data[1], rx_payload.data[2]);
                    if (hid_dev && configured) {
                        uint8_t hid_report[8] = {REPORT_ID, rx_payload.data[1] ? 0xE0 : 0x00, 0x00, rx_payload.data[2] ? 0x04 : 0x00, 0x00, 0x00, 0x00, 0x00};
                        memcpy(last_hid_report, hid_report, 8);
                        new_report_available = true;
                        if (hid_int_ep_write(hid_dev, last_hid_report, 8, NULL)) {
                            LOG_ERR("Failed to submit HID report for GPIO data");
                        } else {
                            LOG_INF("HID report sent to USB host");
                            new_report_available = false;
                        }
                    }
                } else if (packet_type == PACKET_TYPE_BATTERY_VOLTAGE) {
                    uint32_t voltage_mv;
                    memcpy(&voltage_mv, rx_payload.data + 1, sizeof(voltage_mv)); // Skip packet type, read 4 bytes
                    LOG_INF("Received battery voltage: %d mV", voltage_mv);
                    // No HID report for voltage, only terminal logging
                } else {
                    LOG_WRN("Unknown packet type: 0x%02x", packet_type);
                }
            } else {
                LOG_WRN("Invalid payload: length=%d, expected 8 bytes", rx_payload.length);
            }
        }
        break;
    }
}

int main(void) {
    int err;
    LOG_INF("Starting 2.4GHz HID Keyboard Receiver");

    /* Initialize USB HID */
    hid_dev = device_get_binding("HID_0");
    if (!hid_dev) {
        LOG_ERR("Failed to get HID_0 device");
    }
    LOG_INF("USB device found and ready.");

    usb_hid_register_device(hid_dev, hid_report_desc, sizeof(hid_report_desc), &my_hid_ops);
    LOG_INF("HID registered.");

    err = usb_hid_init(hid_dev);
    if (err) {
        LOG_ERR("Failed to init USB HID, err %d", err);
    }
    LOG_INF("HID initialized.");

    LOG_INF("Attempting to enable USB...");
    if (IS_ENABLED(CONFIG_USB_DEVICE_STACK)) {
        usb_dc_set_status_callback(&usb_status_cb);
        err = usb_enable(&usb_status_cb);
        if (err) {
            LOG_ERR("Failed to enable USB, err %d", err);
        }
        LOG_INF("USB enabled successfully.");
    }

    /* Initialize ESB */
    esb_config.protocol = ESB_PROTOCOL_ESB_DPL;
    esb_config.mode = ESB_MODE_PRX;
    esb_config.bitrate = ESB_BITRATE_2MBPS;
    esb_config.payload_length = 8; // Updated to match 8-byte payload
    esb_config.retransmit_count = 3;
    esb_config.event_handler = receiver_esb_event_handler;
    esb_config.selective_auto_ack = true;

    err = esb_init(&esb_config);
    if (err) {
        LOG_ERR("ESB initialization failed, err %d", err);
        return err;
    }

    uint8_t base_addr_0[4] = {0xAB, 0x12, 0xCD, 0x34};
    err = esb_set_base_address_0(base_addr_0);
    if (err) {
        LOG_ERR("Failed to set base address 0, err %d", err);
        return err;
    }

    uint8_t prefixes[8] = {0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8};
    err = esb_set_prefixes(prefixes, 8);
    if (err) {
        LOG_ERR("Failed to set prefixes, err %d", err);
        return err;
    }

    err = esb_start_rx();
    if (err) {
        LOG_ERR("Failed to start ESB RX, err %d", err);
        return err;
    }
    LOG_INF("ESB Receiver initialized and started successfully.");

    while (1) {
        k_sleep(K_SECONDS(1));
    }
    return 0;
}
