#!/usr/bin/env python3
"""Post-test analyzer for SPI stress test logs.

Parses pi_sender.jsonl and firmware_serial.log to produce a summary report.

Usage: python3 spi_stress_report.py <log-dir>
"""
import sys
import os
import json
import re
from collections import defaultdict


def parse_sender_log(path):
    """Parse pi_sender.jsonl and return stats."""
    frames = 0
    ready_timeouts = 0
    total_bytes = 0
    ready_times = []
    speed_tiers = defaultdict(lambda: {"frames": 0, "timeouts": 0})
    first_t = None
    last_t = None

    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            entry = json.loads(line)
            t = entry.get("t", 0)
            tier_key = f"{entry.get('spi_mhz', 0):.0f} MHz"

            if first_t is None:
                first_t = t
            last_t = t

            if entry.get("event") == "ready_timeout":
                ready_timeouts += 1
                speed_tiers[tier_key]["timeouts"] += 1
                continue

            frames += 1
            total_bytes += entry.get("bytes", 0)
            ready_times.append(entry.get("ready_ms", 0))
            speed_tiers[tier_key]["frames"] += 1

    duration = (last_t - first_t) if (first_t is not None and last_t is not None) else 0

    # ready time percentiles
    ready_times.sort()
    avg_ready = sum(ready_times) / len(ready_times) if ready_times else 0
    p99_ready = ready_times[int(len(ready_times) * 0.99)] if ready_times else 0

    return {
        "frames": frames,
        "ready_timeouts": ready_timeouts,
        "total_bytes": total_bytes,
        "duration": duration,
        "avg_ready_ms": avg_ready,
        "p99_ready_ms": p99_ready,
        "avg_fps": frames / duration if duration > 0 else 0,
        "speed_tiers": dict(speed_tiers),
    }


def parse_firmware_log(path):
    """Parse firmware_serial.log for periodic stats lines."""
    # match: [123s] frames:1000 err:0 drop:0 fps:41.1 bytes:7213000 ...
    stats_re = re.compile(
        r'\[(\d+)s\] frames:(\d+) err:(\d+) drop:(\d+) fps:(\d+\.?\d*) bytes:(\d+)'
        r' dma_rem:(\d+) fifo_extra:(\d+)'
        r' err\[sync:(\d+) crc:(\d+) len:(\d+) port:(\d+) align:(\d+) short:(\d+)\]'
    )

    stats = []
    boot_count = 0

    with open(path) as f:
        for line in f:
            if "=== signal" in line:
                boot_count += 1
            m = stats_re.search(line)
            if m:
                stats.append({
                    "uptime_s": int(m.group(1)),
                    "frames": int(m.group(2)),
                    "errors": int(m.group(3)),
                    "drops": int(m.group(4)),
                    "fps": float(m.group(5)),
                    "bytes": int(m.group(6)),
                    "err_sync": int(m.group(9)),
                    "err_crc": int(m.group(10)),
                    "err_len": int(m.group(11)),
                    "err_port": int(m.group(12)),
                    "err_align": int(m.group(13)),
                    "err_short": int(m.group(14)),
                })

    # firmware counters are cumulative from boot. if a boot banner appears
    # in the log, the firmware started fresh during this test -- use the
    # last absolute values. otherwise compute a delta between first and
    # last stats lines to isolate this test's activity.
    DELTA_KEYS = ["frames", "errors", "drops", "bytes",
                  "err_sync", "err_crc", "err_len", "err_port",
                  "err_align", "err_short"]

    if not stats:
        result = {k: 0 for k in DELTA_KEYS}
    elif boot_count > 0:
        # fresh boot captured -- absolute values are this test's totals
        last = stats[-1]
        result = {k: last[k] for k in DELTA_KEYS}
    elif len(stats) >= 2:
        # mid-session -- delta between first and last
        first, last = stats[0], stats[-1]
        result = {k: last[k] - first[k] for k in DELTA_KEYS}
    else:
        result = {k: stats[0][k] for k in DELTA_KEYS}

    last = stats[-1] if stats else {}
    result["avg_fps"] = last.get("fps", 0)
    result["boot_count"] = boot_count
    result["stats_lines"] = len(stats)
    return result


def format_duration(seconds):
    h = int(seconds) // 3600
    m = (int(seconds) % 3600) // 60
    s = int(seconds) % 60
    return f"{h}h {m:02d}m {s:02d}s"


def main():
    if len(sys.argv) < 2:
        print(f"usage: {sys.argv[0]} <log-dir>")
        sys.exit(1)

    log_dir = sys.argv[1]
    sender_path = os.path.join(log_dir, "pi_sender.jsonl")
    serial_path = os.path.join(log_dir, "firmware_serial.log")

    if not os.path.exists(sender_path):
        print(f"error: {sender_path} not found")
        sys.exit(1)

    pi = parse_sender_log(sender_path)

    has_serial = os.path.exists(serial_path)
    fw = parse_firmware_log(serial_path) if has_serial else {}

    # load run parameters if available
    params_path = os.path.join(log_dir, "params.json")
    params = {}
    if os.path.exists(params_path):
        with open(params_path) as f:
            params = json.loads(f.read())

    print(f"=== SPI STRESS TEST REPORT ===")
    print(f"Run: {log_dir}")
    print(f"Duration: {format_duration(pi['duration'])}")
    if params:
        print(f"Config:   {params.get('ports', '?')} ports x {params.get('pixels', '?')} px "
              f"@ {params.get('spi_mhz', '?')} MHz SPI, "
              f"{params.get('target_fps', '?')} fps target")
    print()

    print(f"--- Pi Side ---")
    print(f"Frames sent:         {pi['frames']:,}")
    print(f"READY timeouts:      {pi['ready_timeouts']:,} ({pi['ready_timeouts'] / max(pi['frames'], 1) * 100:.3f}%)")
    print(f"Avg READY wait:      {pi['avg_ready_ms']:.1f} ms (p99: {pi['p99_ready_ms']:.1f} ms)")
    print(f"Avg FPS:             {pi['avg_fps']:.1f}")
    print(f"Total bytes:         {pi['total_bytes']:,}")
    print()

    if has_serial and fw.get("frames", 0) > 0:
        print(f"--- Firmware Side (sampled every 1000 frames) ---")
        print(f"Frames received:     ~{fw['frames']:,}")
        print(f"Frame drops:         {fw['drops']:,} ({fw['drops'] / max(fw['frames'], 1) * 100:.4f}%)")
        print(f"CRC errors:          {fw['err_crc']}")
        print(f"Sync errors:         {fw['err_sync']}")
        print(f"Other errors:        {fw['err_len'] + fw['err_port'] + fw['err_align'] + fw['err_short']}")
        print(f"  length: {fw['err_len']}, port: {fw['err_port']}, "
              f"align: {fw['err_align']}, short: {fw['err_short']}")
        print(f"Avg FPS (firmware):  {fw['avg_fps']:.1f}")
        print(f"Total bytes:         {fw['bytes']:,}")
        if fw["boot_count"] > 1:
            print(f"Watchdog resets:     {fw['boot_count'] - 1} (boot banner appeared {fw['boot_count']} times)")
        print()
    elif has_serial:
        print(f"--- Firmware Side ---")
        print(f"No stats lines found in serial log (firmware may not have the telemetry update)")
        print()

    # per-speed-tier breakdown
    tiers = pi["speed_tiers"]
    if len(tiers) > 1:
        print(f"--- Per Speed Tier ---")
        for tier in sorted(tiers.keys(), key=lambda x: float(x.split()[0])):
            info = tiers[tier]
            print(f"  {tier:>6s}: {info['frames']:,} frames, {info['timeouts']} timeouts")
        print()

    # verdict
    total_errors = fw.get("errors", 0) if fw else 0
    crc_errors = fw.get("err_crc", 0) if fw else 0
    drops = fw.get("drops", 0) if fw else 0
    boot_count = fw.get("boot_count", 1) if fw else 1

    issues = []
    if crc_errors > 0:
        issues.append(f"{crc_errors} CRC errors")
    if pi["ready_timeouts"] > 0:
        issues.append(f"{pi['ready_timeouts']} READY timeouts")
    if boot_count > 1:
        issues.append(f"{boot_count - 1} watchdog resets")
    if pi["frames"] > 0 and drops / max(pi["frames"], 1) > 0.0001:
        issues.append(f"drop rate {drops / pi['frames'] * 100:.4f}%")

    print(f"--- Verdict ---")
    if not issues:
        print(f"PASS: no CRC errors, no timeouts, no resets")
    else:
        print(f"ISSUES: {', '.join(issues)}")


if __name__ == "__main__":
    main()
