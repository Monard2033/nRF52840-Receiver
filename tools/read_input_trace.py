#!/usr/bin/env python3
"""Freeze and drain the Receiver's diagnostic HID trace on Windows."""

from __future__ import annotations

import argparse
import csv
import ctypes
from ctypes import wintypes
from datetime import datetime
from pathlib import Path
import struct
import sys


REPORT_ID = 0x05
REPORT_LEN = 21
TRACE_CMD_CLEAR = 0x01
TRACE_CMD_FREEZE = 0x02
TRACE_CMD_RESUME = 0x03

DIGCF_PRESENT = 0x00000002
DIGCF_DEVICEINTERFACE = 0x00000010
GENERIC_READ = 0x80000000
GENERIC_WRITE = 0x40000000
FILE_SHARE_READ = 0x00000001
FILE_SHARE_WRITE = 0x00000002
OPEN_EXISTING = 3

STAGES = {
    1: "ESB_RX",
    2: "HID_QUEUED",
    3: "HID_QUEUE_OVERFLOW",
    4: "DUPLICATE_DROP",
    5: "SEQUENCE_DROP",
    6: "USB_NOT_READY_DROP",
    7: "HID_SUBMIT_OK",
    8: "HID_SUBMIT_BUSY",
    9: "HID_SUBMIT_ERROR",
    10: "HID_COMPLETE",
}

PACKET_TYPES = {1: "KEYBOARD", 2: "CONSUMER"}
MODIFIERS = (
    (0x01, "LCTRL"), (0x02, "LSHIFT"), (0x04, "LALT"), (0x08, "LGUI"),
    (0x10, "RCTRL"), (0x20, "RSHIFT"), (0x40, "RALT"), (0x80, "RGUI"),
)


class GUID(ctypes.Structure):
    _fields_ = [
        ("Data1", wintypes.DWORD),
        ("Data2", wintypes.WORD),
        ("Data3", wintypes.WORD),
        ("Data4", wintypes.BYTE * 8),
    ]


class SP_DEVICE_INTERFACE_DATA(ctypes.Structure):
    _fields_ = [
        ("cbSize", wintypes.DWORD),
        ("InterfaceClassGuid", GUID),
        ("Flags", wintypes.DWORD),
        ("Reserved", ctypes.c_size_t),
    ]


hid = ctypes.WinDLL("hid", use_last_error=True)
setupapi = ctypes.WinDLL("setupapi", use_last_error=True)
kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)

kernel32.CreateFileW.argtypes = [
    wintypes.LPCWSTR, wintypes.DWORD, wintypes.DWORD, wintypes.LPVOID,
    wintypes.DWORD, wintypes.DWORD, wintypes.HANDLE,
]
kernel32.CreateFileW.restype = wintypes.HANDLE
kernel32.CloseHandle.argtypes = [wintypes.HANDLE]
kernel32.CloseHandle.restype = wintypes.BOOL
hid.HidD_GetHidGuid.argtypes = [ctypes.POINTER(GUID)]
hid.HidD_GetFeature.argtypes = [wintypes.HANDLE, wintypes.LPVOID, wintypes.ULONG]
hid.HidD_GetFeature.restype = wintypes.BOOLEAN
hid.HidD_SetFeature.argtypes = [wintypes.HANDLE, wintypes.LPVOID, wintypes.ULONG]
hid.HidD_SetFeature.restype = wintypes.BOOLEAN
setupapi.SetupDiGetClassDevsW.argtypes = [
    ctypes.POINTER(GUID), wintypes.LPCWSTR, wintypes.HWND, wintypes.DWORD,
]
setupapi.SetupDiGetClassDevsW.restype = wintypes.HANDLE
setupapi.SetupDiEnumDeviceInterfaces.argtypes = [
    wintypes.HANDLE, wintypes.LPVOID, ctypes.POINTER(GUID), wintypes.DWORD,
    ctypes.POINTER(SP_DEVICE_INTERFACE_DATA),
]
setupapi.SetupDiEnumDeviceInterfaces.restype = wintypes.BOOL
setupapi.SetupDiGetDeviceInterfaceDetailW.argtypes = [
    wintypes.HANDLE, ctypes.POINTER(SP_DEVICE_INTERFACE_DATA),
    wintypes.LPVOID, wintypes.DWORD, ctypes.POINTER(wintypes.DWORD),
    wintypes.LPVOID,
]
setupapi.SetupDiGetDeviceInterfaceDetailW.restype = wintypes.BOOL
setupapi.SetupDiDestroyDeviceInfoList.argtypes = [wintypes.HANDLE]
setupapi.SetupDiDestroyDeviceInfoList.restype = wintypes.BOOL

INVALID_HANDLE_VALUE = wintypes.HANDLE(-1).value


def get_feature(handle: int) -> bytes | None:
    buffer = (ctypes.c_ubyte * REPORT_LEN)()
    buffer[0] = REPORT_ID
    if not hid.HidD_GetFeature(handle, ctypes.byref(buffer), REPORT_LEN):
        return None
    if buffer[0] != REPORT_ID or buffer[1] != 1:
        return None
    return bytes(buffer)


def set_command(handle: int, command: int) -> bool:
    buffer = (ctypes.c_ubyte * REPORT_LEN)()
    buffer[0] = REPORT_ID
    buffer[1] = command
    return bool(hid.HidD_SetFeature(handle, ctypes.byref(buffer), REPORT_LEN))


def find_receiver() -> int | None:
    hid_guid = GUID()
    hid.HidD_GetHidGuid(ctypes.byref(hid_guid))
    dev_info = setupapi.SetupDiGetClassDevsW(
        ctypes.byref(hid_guid), None, None,
        DIGCF_PRESENT | DIGCF_DEVICEINTERFACE,
    )
    if dev_info == INVALID_HANDLE_VALUE:
        return None

    try:
        index = 0
        while True:
            interface = SP_DEVICE_INTERFACE_DATA()
            interface.cbSize = ctypes.sizeof(interface)
            if not setupapi.SetupDiEnumDeviceInterfaces(
                dev_info, None, ctypes.byref(hid_guid), index,
                ctypes.byref(interface),
            ):
                break
            index += 1

            required = wintypes.DWORD()
            setupapi.SetupDiGetDeviceInterfaceDetailW(
                dev_info, ctypes.byref(interface), None, 0,
                ctypes.byref(required), None,
            )
            detail = ctypes.create_string_buffer(required.value)
            struct.pack_into(
                "I", detail, 0,
                8 if ctypes.sizeof(ctypes.c_void_p) == 8 else 6,
            )
            if not setupapi.SetupDiGetDeviceInterfaceDetailW(
                dev_info, ctypes.byref(interface), detail, required.value,
                None, None,
            ):
                continue

            path = ctypes.wstring_at(ctypes.addressof(detail) + 4)
            handle = kernel32.CreateFileW(
                path, GENERIC_READ | GENERIC_WRITE,
                FILE_SHARE_READ | FILE_SHARE_WRITE, None,
                OPEN_EXISTING, 0, None,
            )
            if handle == INVALID_HANDLE_VALUE:
                continue
            normalized_path = path.lower()
            if ("vid_1b4f&pid_0001" in normalized_path and
                    "&col05" in normalized_path):
                return handle
            kernel32.CloseHandle(handle)
    finally:
        setupapi.SetupDiDestroyDeviceInfoList(dev_info)
    return None


def signed_result(value: int) -> int:
    return value - 256 if value >= 128 else value


def keyboard_description(data: bytes) -> str:
    modifiers = "+".join(name for bit, name in MODIFIERS if data[0] & bit)
    keys = []
    for usage in data[2:]:
        if usage == 0:
            continue
        if 0x04 <= usage <= 0x1D:
            keys.append(chr(ord("A") + usage - 0x04))
        else:
            keys.append(f"0x{usage:02X}")
    return f"mods={modifiers or '-'} keys={'+'.join(keys) or '-'}"


def drain_trace(handle: int) -> tuple[list[dict[str, object]], int]:
    if not set_command(handle, TRACE_CMD_FREEZE):
        raise RuntimeError("Receiver accepted neither the TRACE FREEZE command nor report ID 5")

    rows: list[dict[str, object]] = []
    overwritten = 0
    while True:
        report = get_feature(handle)
        if report is None:
            raise RuntimeError("TRACE GetFeature failed while draining the ring")
        flags = report[2]
        remaining = report[3]
        overwritten = max(overwritten, report[4])
        if not flags & 0x01:
            break

        timestamp_ms, stage, packet_type, sequence, result, data = struct.unpack(
            "<IBBBb8s", report[5:21]
        )
        description = (
            keyboard_description(data) if packet_type == 1
            else f"usage=0x{data[1]:02X}{data[0]:02X}"
        )
        rows.append({
            "timestamp_ms": timestamp_ms,
            "stage": STAGES.get(stage, f"UNKNOWN_{stage}"),
            "packet_type": PACKET_TYPES.get(packet_type, f"TYPE_{packet_type}"),
            "sequence": sequence,
            "result": signed_result(result & 0xFF),
            "data_hex": " ".join(f"{byte:02X}" for byte in data),
            "description": description,
        })
        if remaining == 0:
            break
    return rows, overwritten


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Freeze and read the diagnostic input trace from Receiver"
    )
    parser.add_argument("--output", type=Path, help="CSV output path")
    args = parser.parse_args()

    handle = find_receiver()
    if handle is None:
        print("Receiver diagnostic HID report ID 5 was not found.", file=sys.stderr)
        return 2

    try:
        try:
            rows, overwritten = drain_trace(handle)
        except RuntimeError as error:
            print(f"Trace read failed: {error}", file=sys.stderr)
            return 3
        output = args.output or Path(
            f"receiver_input_trace_{datetime.now():%Y%m%d_%H%M%S}.csv"
        )
        with output.open("w", newline="", encoding="utf-8-sig") as stream:
            writer = csv.DictWriter(stream, fieldnames=[
                "timestamp_ms", "delta_ms", "stage", "packet_type",
                "sequence", "result", "data_hex", "description",
            ])
            writer.writeheader()
            first = rows[0]["timestamp_ms"] if rows else 0
            for row in rows:
                out_row = dict(row)
                out_row["delta_ms"] = int(row["timestamp_ms"]) - int(first)
                writer.writerow(out_row)
                print(
                    f"{out_row['delta_ms']:6d} ms  {row['stage']:<20} "
                    f"seq={row['sequence']:3d}  {row['data_hex']}  "
                    f"{row['description']}  rc={row['result']}"
                )
        print(f"\nSaved {len(rows)} trace records to: {output.resolve()}")
        print(f"Ring overwrites before freeze: {overwritten}")
        return 0
    finally:
        set_command(handle, TRACE_CMD_CLEAR)
        set_command(handle, TRACE_CMD_RESUME)
        kernel32.CloseHandle(handle)


if __name__ == "__main__":
    raise SystemExit(main())
