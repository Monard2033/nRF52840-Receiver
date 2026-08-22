#!/usr/bin/env python3
"""Continuous live trace monitor for Receiver (streams for N seconds)."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
from pathlib import Path
import sys
import time

from read_input_trace import drain_trace, find_receiver, set_command, TRACE_CMD_CLEAR, TRACE_CMD_RESUME


def main() -> int:
    parser = argparse.ArgumentParser(description="Continuous live trace monitor")
    parser.add_argument("--duration", type=int, default=120, help="Duration in seconds (default: 120)")
    parser.add_argument("--output", type=Path, help="CSV output path")
    args = parser.parse_args()

    handle = find_receiver()
    if handle is None:
        print("Receiver diagnostic HID report ID 5 was not found.", file=sys.stderr)
        return 2

    # Clear initial buffer
    set_command(handle, TRACE_CMD_CLEAR)
    set_command(handle, TRACE_CMD_RESUME)

    output_path = args.output or Path(f"receiver_continuous_trace_{datetime.now():%Y%m%d_%H%M%S}.csv")
    print(f"=== Starting continuous 2-minute monitor ({args.duration}s) ===")
    print(f"Saving live trace to: {output_path.resolve()}\n")
    sys.stdout.flush()

    all_rows: list[dict[str, object]] = []
    total_overwritten = 0
    start_time = time.time()
    last_print = start_time

    try:
        while time.time() - start_time < args.duration:
            time.sleep(0.15)  # drain every 150 ms
            rows, over = drain_trace(handle)
            if rows:
                all_rows.extend(rows)
            total_overwritten += over

            now = time.time()
            if now - last_print >= 5.0:
                elapsed = int(now - start_time)
                print(f"[{elapsed:3d}s / {args.duration}s] Captured {len(all_rows)} events...")
                sys.stdout.flush()
                last_print = now

        # Final drain
        rows, over = drain_trace(handle)
        if rows:
            all_rows.extend(rows)
        total_overwritten += over

    finally:
        set_command(handle, TRACE_CMD_CLEAR)
        set_command(handle, TRACE_CMD_RESUME)

    # Save CSV
    with output_path.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=[
            "timestamp_ms", "delta_ms", "stage", "packet_type",
            "sequence", "result", "data_hex", "description",
        ])
        writer.writeheader()
        first_ts = all_rows[0]["timestamp_ms"] if all_rows else 0
        for row in all_rows:
            out_row = dict(row)
            out_row["delta_ms"] = int(row["timestamp_ms"]) - int(first_ts)
            writer.writerow(out_row)

    # Compute statistics
    esb_rx_count = sum(1 for r in all_rows if r["stage"] == "ESB_RX")
    hid_submit_ok = sum(1 for r in all_rows if r["stage"] == "HID_SUBMIT_OK")
    hid_busy = sum(1 for r in all_rows if r["stage"] == "HID_SUBMIT_BUSY")
    hid_overflow = sum(1 for r in all_rows if r["stage"] == "HID_QUEUE_OVERFLOW")
    dup_drops = sum(1 for r in all_rows if r["stage"] == "DUPLICATE_DROP")
    seq_drops = sum(1 for r in all_rows if r["stage"] == "SEQUENCE_DROP")

    print("\n" + "=" * 60)
    print("=== 2-MINUTE CONTINUOUS MONITORING SUMMARY ===")
    print("=" * 60)
    print(f"Total Trace Records Captured : {len(all_rows)}")
    print(f"Total Radio Packets (ESB_RX) : {esb_rx_count}")
    print(f"USB Reports Delivered (OK)   : {hid_submit_ok}")
    print(f"USB Busy Events (Stalls)     : {hid_busy}")
    print(f"Queue Overruns               : {hid_overflow}")
    print(f"Sequence Drops (Lost Order)  : {seq_drops}")
    print(f"Duplicate Drops              : {dup_drops}")
    print(f"Ring Overwrites              : {total_overwritten}")
    print("=" * 60)
    print(f"Complete CSV saved to: {output_path.resolve()}")
    sys.stdout.flush()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
