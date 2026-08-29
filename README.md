# nRF52840 USB Dongle Wireless Receiver

This firmware runs on the **nRF52840 USB Dongle** (Holyiot nRF52840 Dongle / Nordic PCA10059). It operates as the dedicated 2.4 GHz Enhanced ShockBurst (ESB) radio receiver and high-speed USB HID composite bridge for the custom wireless keyboard system, delivering true **1000 Hz (1 ms) sub-millisecond input latency** to Windows with zero packet loss.

---

## Current Release Baseline (v1.0-stable)

- **Branch**: `release/v1.0-stable` across all three project repositories:
  - [RP2040 Keyboard Controller](https://github.com/Monard2033/RaspberryPicoUSBHost)
  - [nRF52840 Transmitter](https://github.com/Monard2033/nRF52840-Transmitter)
  - [nRF52840 Receiver](https://github.com/Monard2033/nRF52840-Receiver)
- **Primary Binary Artifact**: [`firmware/receiver.hex`](firmware/receiver.hex)
- **SHA-256 Checksum**: `16DF6B5D9A7FEDB02D9F04BA0057EA2F9F36F43DF78791B8FB3F22599DBE16F4`

---

## Key Features & Architecture

1. **Ultra-Low Latency 1000 Hz (1 ms) USB Delivery**:
   - Enumerates as a high-speed USB Full-Speed HID device with `bInterval = 1` ($1.0\text{ ms}$).
   - Non-blocking HID submit queue forwards key presses, multi-key bursts, and multimedia events to Windows in $< 0.4\text{ ms}$ total end-to-end flight time.
2. **High-Power 2.4 GHz ESB Radio (+8 dBm)**:
   - Configured at maximum hardware output power (`ESB_TX_POWER_8DBM` / $+8\text{ dBm} \approx 6.3\text{ mW}$) and 2 Mbps bitrate on **Channel 90 ($2490\text{ MHz}$)**.
   - Operating on Channel 90 places the wireless link +17 MHz above standard Wi-Fi Channel 11 and +10 MHz above Bluetooth, ensuring clean air and high interference immunity.
3. **5-in-1 Composite HID Architecture**:
   - Single physical USB endpoint exposes 5 distinct logical HID collections cleanly separated by Report IDs.
4. **Bidirectional Lock-State Synchronization**:
   - Captures Windows NumLock, CapsLock, and ScrollLock LED states from the host OS and embeds them into hardware radio ACKs transmitted back to the keyboard with epoch sequence tracking.
5. **Real-Time Battery Telemetry Engine**:
   - Receives rolling voltage, percentage, and charging/discharging power states from the RP2040.
   - Exposes live telemetry via Vendor Feature Reports for native Windows tray monitoring (`WirelessKeyboardTray.exe`) without filtering or sequence lockups.
6. **Zero-Overhead Diagnostic Trace Buffer**:
   - Integrated circular ring buffer records timestamped USB/Radio events at sub-millisecond precision.
   - Queryable on demand via CLI tools with zero impact on the 1000 Hz input hot-path.

---

## 5-in-1 HID Collections Specification

The Receiver USB descriptor integrates 5 specialized collections recognizable in Windows Device Manager:

| Report ID | Collection Name | Description & Capabilities |
| :---: | :--- | :--- |
| **`1`** | **`A4TECH USB Receiver`** | **1000 Hz Standard Keyboard**: Full 6KRO + Modifier bitmap supporting standard boot and report protocols, rapid rollover containment, and ErrorRollOver recovery. |
| **`2`** | **`A4TECH Consumer Media Control`** | **Multimedia Controller**: Dedicated 16-bit consumer control usage for Volume Up/Down, Mute, Play/Pause, Next Track, and Previous Track. |
| **`3`** | **`A4TECH Battery & Power Management`** | **Battery Telemetry Vendor Interface**: 8-byte feature payload containing battery percentage, operating state (Discharging, Charging CC/CV, Full), raw millivolts, and live telemetry age. |
| **`4`** | **`A4TECH Wireless Configuration Interface`** | **OTA DFU Session Controller**: Vendor-defined bidirectional pipe for over-the-air firmware updates and bootloader handshakes. |
| **`5`** | **`A4TECH 1000Hz Diagnostic Interface`** | **Diagnostic Trace Ring Buffer**: Read-on-demand 32-byte trace record stream for real-time latency analysis and sequence validation. |

---

## Recommended Hardware & Procurement

For optimal performance, RF stability, and direct USB plug-and-play operation:

- **Recommended Hardware**: **Holyiot nRF52840 USB Dongle** or **Nordic Semiconductor nRF52840 Dongle (PCA10059)**.
- **Microcontroller**: Nordic nRF52840 (ARM Cortex-M4F @ 64 MHz with FPU, 1 MB Flash, 256 KB RAM).
- **Antenna**: Integrated high-efficiency 2.4 GHz PCB trace antenna matched for Channel 90.
- **Onboard Peripherals**: User programmable RGB LED + Green LED, hardware reset button for DFU bootloader.

---

## Flashing & Programming Guide

The nRF52840 Dongle comes with a built-in Open Bootloader (DFU). No external JTAG/SWD programmer is required.

### Flashing via nRF Connect for Desktop (GUI):

1. Download and install [nRF Connect for Desktop](https://www.nordicsemi.com/Products/Development-tools/nRF-Connect-for-Desktop).
2. Install and launch the **Programmer** tool within nRF Connect.
3. Insert the nRF52840 USB Dongle into a PC USB port.
4. Press the **small lateral reset button** on the dongle (the red LED will begin pulsing slowly in DFU mode).
5. In the top-left dropdown of the Programmer app, select the detected Nordic Dongle.
6. Click **Add file** $\rightarrow$ **Browse** and select [`firmware/receiver.hex`](firmware/receiver.hex).
7. Click **Write** to flash the firmware.
8. The dongle will automatically restart and enumerate in Windows as `A4TECH USB Receiver` at 1000 Hz!

---

## Critical `prj.conf` Configuration Directives

The following Zephyr / Nordic Connect SDK (NCS v3.4.0) configuration options must be strictly maintained:

```ini
# Silent release build: USB HID only, zero UART/logging CPU overhead
CONFIG_LOG=n
CONFIG_CONSOLE=n
CONFIG_BOOT_BANNER=n
CONFIG_NCS_BOOT_BANNER=n
CONFIG_PRINTK=n
CONFIG_LOG_BACKEND_UART=n
CONFIG_LOG_BACKEND_RTT=n
CONFIG_UART_CONSOLE=n
CONFIG_SERIAL=n

# ESB High-Priority Radio Subsystem
CONFIG_ESB=y
CONFIG_CLOCK_CONTROL=y
CONFIG_ESB_CLOCK_INIT=y
CONFIG_ESB_RADIO_IRQ_PRIORITY=0
CONFIG_ESB_PIPE_COUNT=8

# USB HID Device Stack & 1000 Hz Polling Rate
CONFIG_USB_DEVICE_STACK=y
CONFIG_USB_DEVICE_INITIALIZE_AT_BOOT=n
CONFIG_USB_COMPOSITE_DEVICE=n
CONFIG_USB_DEVICE_HID=y
CONFIG_ENABLE_HID_INT_OUT_EP=y
CONFIG_USB_HID_POLL_INTERVAL_MS=1
CONFIG_USB_HID_BOOT_PROTOCOL=y
CONFIG_USB_DEVICE_OS_DESC=y

# Device Identification
CONFIG_USB_DEVICE_VID=0x1B4F
CONFIG_USB_DEVICE_PID=0x0001
CONFIG_USB_DEVICE_PRODUCT="A4TECH Receiver"
CONFIG_USB_DEVICE_MANUFACTURER="Monard"
```

---

## Included Diagnostic & Management Tools

Located in the [`tools/`](tools/) directory:

- **`read_battery.py`**: Queries the live battery telemetry cache over USB and displays percentage, voltage ($0.001\text{ V}$ accuracy), charging state, and telemetry age:
  ```powershell
  python tools/read_battery.py
  ```
- **`read_input_trace.py`**: Extracts the 256-record diagnostic ring buffer from the Receiver, displays latency deltas, and saves a CSV log for sequence verification:
  ```powershell
  python tools/read_input_trace.py
  ```
- **`register_a4tech_names.ps1`**: PowerShell script to register custom friendly device names in the Windows registry for Device Manager.
