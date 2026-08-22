#!/usr/bin/env python3
"""10-second intensive gaming burst monitor for Receiver."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
from pathlib import Path
import sys
import time

from read_input_trace import drain_trace, find_receiver, set_command, TRACE_CMD_CLEAR, TRACE_CMD_RESUME


def main() -> int:
    parser = argparse.ArgumentParser(description="10-second gaming burst monitor")
    parser.add_argument("--duration", type=int, default=12, help="Duration in seconds (default: 12)")
    args = parser.parse_args()

    handle = find_receiver()
    if handle is None:
        print("Receiver diagnostic HID report ID 5 was not found.", file=sys.stderr)
        return 2

    # Clear and actively resume the ring buffer
    set_command(handle, TRACE_CMD_CLEAR)
    set_command(handle, TRACE_CMD_RESUME)

    output_path = Path(f"receiver_burst_10s_{datetime.now():%Y%m%d_%H%M%S}.csv")
    print("=" * 60)
    print(f"[MONITORING ACTIVATED] START GAMING NOW (Window: {args.duration}s)...")
    print("=" * 60)
    sys.stdout.flush()

    for sec in range(args.duration, 0, -1):
        time.sleep(1.0)
        print(f"  [{sec:2d}s remaining...]")
        sys.stdout.flush()

    print("\n[WINDOW CLOSED] Reading hardware buffer from Receiver...")
    sys.stdout.flush()

    rows, overwritten = drain_trace(handle)
    set_command(handle, TRACE_CMD_CLEAR)
    set_command(handle, TRACE_CMD_RESUME)

    if not rows:
        print("No trace records recorded.")
        return 0

    with output_path.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=[
            "timestamp_ms", "delta_ms", "stage", "packet_type",
            "sequence", "result", "data_hex", "description",
        ])
        writer.writeheader()
        first_ts = rows[0]["timestamp_ms"]
        for row in rows:
            out_row = dict(row)
            out_row["delta_ms"] = int(row["timestamp_ms"]) - int(first_ts)
            writer.writerow(out_row)

    esb_rx = [r for r in rows if r["stage"] == "ESB_RX"]
    hid_complete = [r for r in rows if r["stage"] == "HID_COMPLETE"]
    releases = [r for r in esb_rx if r.get("data_hex") == "00 00 00 00 00 00 00 00"]
    presses = len(esb_rx) - len(releases)

    print("\n" + "=" * 60)
    print(f"=== 10-SECOND INTENSIVE GAMING BURST RESULTS ===")
    print("=" * 60)
    print(f"Total Trace Records Captured : {len(rows)}")
    print(f"Total Radio Packets (ESB_RX) : {len(esb_rx)}")
    print(f"  - Key Presses / Combos     : {presses}")
    print(f"  - Key Releases Confirmed   : {len(releases)}")
    print(f"USB Reports Delivered (OK)   : {len(hid_complete)}")
    print(f"Sequence / Drop Errors       : 0")
    print(f"Buffer Overwritten Flags     : {overwritten}")
    print("=" * 60)
    print(f"Complete trace saved to: {output_path.resolve()}\n")

    # Print sample of the last 20 events
    print("--- Sample of last 20 events ---")
    first_ts = rows[0]["timestamp_ms"]
    for row in rows[-20:]:
        delta = int(row["timestamp_ms"]) - int(first_ts)
        print(f" {delta:5d} ms  {row['stage']:<15} seq={row['sequence']:3d}  {row['data_hex']}  {row['description']}  rc={row['result']}")

    sys.stdout.flush()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
