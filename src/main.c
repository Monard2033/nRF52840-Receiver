/*
 * Copyright (c) 2024 Monard2033
 * SPDX-License-License-Identifier: Apache-2.0
 */

 #include <zephyr/kernel.h>
 #include <zephyr/usb/usb_device.h>
 #include <zephyr/usb/class/usb_hid.h>
 #include <zephyr/logging/log.h>
 #include <zephyr/device.h>
 #include <esb.h>
 #include <string.h>
 
 LOG_MODULE_REGISTER(RECEIVER, LOG_LEVEL_INF);
 
 /* --- ESB Configuration --- */
 static struct esb_payload rx_payload;
 static struct esb_config esb_config = ESB_DEFAULT_CONFIG;
 
 /* --- USB HID Configuration --- */
 static const struct device *hid_dev;
 static K_SEM_DEFINE(usb_configured_sem, 0, 1);
 static volatile bool configured = true; /* Set by default for testing */
 
 /* Key Configuration */
 #define KEY_CTRL_CODE_MIN 224
 #define KEY_CTRL_CODE_MAX 231
 #define KEY_CODE_MIN      0
 #define KEY_CODE_MAX      101
 #define KEY_PRESS_MAX     6
 #define INPUT_REPORT_KEYS_MAX_LEN (1 + 1 + KEY_PRESS_MAX)
 
 BUILD_ASSERT((KEY_CTRL_CODE_MAX - KEY_CTRL_CODE_MIN) + 1 == 8);
 
 /* HID Report Descriptor for a standard keyboard */
 static const uint8_t hid_report_desc[] = {
     0x05, 0x01, 0x09, 0x06, 0xA1, 0x01, 0x75, 0x01,
     0x95, 0x08, 0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7,
     0x15, 0x00, 0x25, 0x01, 0x81, 0x02, 0x95, 0x01,
     0x75, 0x08, 0x81, 0x01, 0x95, 0x06, 0x75, 0x08,
     0x15, 0x00, 0x25, 0x65, 0x05, 0x07, 0x19, 0x00,
     0x29, 0x65, 0x81, 0x00, 0xC0
 };
 
 /* Keyboard State */
 static struct keyboard_state {
     uint8_t ctrl_keys_state;
     uint8_t keys_state[KEY_PRESS_MAX];
 } hid_keyboard_state;
 
 static void usb_status_cb(enum usb_dc_status_code status, const uint8_t *param)
 {
     ARG_UNUSED(param);
     LOG_DBG("USB status callback: status=%d", status);
     switch (status) {
     case USB_DC_CONFIGURED:
         LOG_INF("USB_DC_CONFIGURED received, setting configured = true");
         configured = true;
         k_sem_give(&usb_configured_sem);
         break;
     case USB_DC_DISCONNECTED:
         LOG_INF("USB_DC_DISCONNECTED received, setting configured = false");
         configured = false;
         break;
     case USB_DC_RESET:
         LOG_INF("USB_DC_RESET received, setting configured = false");
         configured = false;
         break;
     case USB_DC_SUSPEND:
         LOG_INF("USB_DC_SUSPEND received");
         break;
     case USB_DC_RESUME:
         LOG_INF("USB_DC_RESUME received");
         break;
     default:
         LOG_DBG("Unknown USB status code: %d", status);
         break;
     }
 }
 
 void receiver_esb_event_handler(struct esb_evt const *event)
 {
     switch (event->evt_id) {
     case ESB_EVENT_TX_SUCCESS:
         LOG_INF("TX success (unexpected in PRX mode)");
         break;
     case ESB_EVENT_TX_FAILED:
         LOG_ERR("TX failed (unexpected in PRX mode)");
         break;
     case ESB_EVENT_RX_RECEIVED:
        if (esb_read_rx_payload(&rx_payload) == 0) {
            LOG_INF("Received Payload (length: %d bytes)", rx_payload.length);
            LOG_INF("Data: D+ = %d, D- = %d", rx_payload.data[0], rx_payload.data[1]);
            LOG_HEXDUMP_INF(rx_payload.data, rx_payload.length, "Raw Payload");
            if (rx_payload.length == 8) {
                LOG_INF("Processing HID report (8 bytes)");
                if (hid_dev && configured) {
                    uint8_t hid_report[8] = {0};
                    uint8_t release_report[8] = {0};
                    memcpy(hid_report, rx_payload.data, 8);
                    hid_int_ep_write(hid_dev, hid_report, 8, NULL);
                    hid_int_ep_write(hid_dev, release_report, 8, NULL);
            } else if (rx_payload.length == 2) {
                LOG_INF("Received test payload (2 bytes): D+ = %d, D- = %d", rx_payload.data[0], rx_payload.data[1]);
            } else {
                LOG_WRN("Unexpected payload length: %d", rx_payload.length);
                LOG_HEXDUMP_INF(rx_payload.data, rx_payload.length, "Unexpected Payload");
            }
        } else {
            LOG_ERR("Failed to read RX payload");
        }
        break;
        }
    }
}
 
 int main(void)
 {
     int err;
     LOG_INF("Starting 2.4GHz HID Keyboard sample");
     /* Initialize USB HID */
     hid_dev = device_get_binding("HID_0");
     if (!hid_dev) {
         LOG_ERR("Failed to get USB HID device 'HID_0'. Exiting.");
         return 0;
     }
     LOG_INF("USB device node acquired, checking readiness...");
     if (!device_is_ready(hid_dev)) {
         LOG_ERR("USB device not ready. Exiting.");
         return 0;
     }
     LOG_INF("USB device found and ready.");
 
     usb_hid_register_device(hid_dev, hid_report_desc, sizeof(hid_report_desc), NULL);
     LOG_INF("HID registered.");
 
     err = usb_hid_init(hid_dev);
     if (err) {
         LOG_ERR("Failed to init USB HID, err %d", err);
         return 0;
     }
     LOG_INF("HID initialized.");
 
     LOG_INF("Waiting for hardware readiness before enabling USB...");
     k_sleep(K_MSEC(10));
     LOG_INF("Attempting to enable USB...");
     if (IS_ENABLED(CONFIG_USB_DEVICE_STACK)) {
         usb_dc_set_status_callback(&usb_status_cb);
         err = usb_enable(NULL);
         if (err) {
             LOG_ERR("Failed to enable USB, err %d", err);
             return 0;
         }
         LOG_INF("USB enabled successfully.");
     }
     k_sleep(K_MSEC(350));
 
     /* Initialize ESB */
     esb_config.protocol = ESB_PROTOCOL_ESB_DPL;
     esb_config.mode = ESB_MODE_PRX;
     esb_config.bitrate = ESB_BITRATE_2MBPS;
     esb_config.payload_length = 8;
     esb_config.retransmit_count = 3;
     esb_config.event_handler = receiver_esb_event_handler;
 
     err = esb_init(&esb_config);
     if (err) {
         LOG_ERR("ESB initialization failed, err %d", err);
         return 0;
     }
 
     uint8_t base_addr_0[4] = {0xAB, 0x12, 0xCD, 0x34};
     err = esb_set_base_address_0(base_addr_0);
     if (err) {
         LOG_ERR("Failed to set base address 0, err %d", err);
         return 0;
     }
 
     uint8_t prefixes[8] = {0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8};
     err = esb_set_prefixes(prefixes, 8);
     if (err) {
         LOG_ERR("Failed to set prefixes, err %d", err);
         return 0;
     }
 
     err = esb_start_rx();
     if (err) {
         LOG_ERR("Failed to start ESB RX, err %d", err);
         return 0;
     }
     LOG_INF("ESB Receiver initialized and started successfully.");

     while (1) {
         k_sleep(K_SECONDS(1));
     }
     return 0;
 }