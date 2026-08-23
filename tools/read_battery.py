#!/usr/bin/env python3
"""Read live battery telemetry (percentage, voltage, charging state) from A4TECH Receiver."""

from __future__ import annotations

import ctypes
from ctypes import wintypes
from pathlib import Path
import struct
import sys

sys.path.insert(0, str(Path(__file__).parent.resolve()))
from read_input_trace import GUID, SP_DEVICE_INTERFACE_DATA, setupapi, DIGCF_PRESENT, DIGCF_DEVICEINTERFACE

hid = ctypes.windll.hid
kernel32 = ctypes.windll.kernel32

STATE_NAMES = {
    0: "IDLE (discharging)",
    1: "CHARGING (breathing LED)",
    2: "UNPLUGGED (discharging)",
    3: "FULL (100% charged)",
    4: "UNKNOWN / BOOT",
}


def read_battery():
    hid_guid = GUID()
    hid.HidD_GetHidGuid(ctypes.byref(hid_guid))
    dev_info = setupapi.SetupDiGetClassDevsW(
        ctypes.byref(hid_guid), None, None, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE
    )

    idx = 0
    while True:
        iface = SP_DEVICE_INTERFACE_DATA()
        iface.cbSize = ctypes.sizeof(iface)
        if not setupapi.SetupDiEnumDeviceInterfaces(dev_info, None, ctypes.byref(hid_guid), idx, ctypes.byref(iface)):
            break
        idx += 1
        req = wintypes.DWORD()
        setupapi.SetupDiGetDeviceInterfaceDetailW(dev_info, ctypes.byref(iface), None, 0, ctypes.byref(req), None)
        buf = ctypes.create_string_buffer(req.value)
        struct.pack_into("I", buf, 0, 8 if ctypes.sizeof(ctypes.c_void_p) == 8 else 6)
        if setupapi.SetupDiGetDeviceInterfaceDetailW(dev_info, ctypes.byref(iface), buf, req.value, None, None):
            path = ctypes.wstring_at(ctypes.addressof(buf) + 4)
            if "vid_1b4f&pid_0001" in path.lower() and "&col03" in path.lower():
                h = kernel32.CreateFileW(path, 0xC0000000, 3, None, 3, 0, None)
                if h != -1:
                    rep = (ctypes.c_ubyte * 9)()
                    rep[0] = 3
                    ok = hid.HidD_GetFeature(h, ctypes.byref(rep), 9)
                    kernel32.CloseHandle(h)
                    setupapi.SetupDiDestroyDeviceInfoList(dev_info)

                    if ok and rep[0] == 3:
                        pct = rep[1]
                        state = rep[2]
                        mv = rep[3] | (rep[4] << 8)
                        seq = rep[5]
                        flags = rep[6]
                        age = rep[7] | (rep[8] << 8)
                        valid = bool(flags & 0x01)

                        print("=" * 55)
                        print("[BATTERY] A4TECH WIRELESS KEYBOARD - LIVE STATUS")
                        print("=" * 55)
                        print(f"  Nivel Baterie : {pct}%")
                        print(f"  Tensiune Celula: {mv} mV ({mv / 1000.0:.3f} V)")
                        print(f"  Stare Incarcare: {STATE_NAMES.get(state, f'State {state}')}")
                        print(f"  Varsta Masurare: {age} secunde in urma (Seq #{seq})")
                        print(f"  Date Valide   : {'DA' if valid else 'NU'}")
                        print("=" * 55)
                        return 0

    setupapi.SetupDiDestroyDeviceInfoList(dev_info)
    print("Nu s-a gasit interfata HID de Baterie (COL03).", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(read_battery())
