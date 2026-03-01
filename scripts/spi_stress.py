#!/usr/bin/env python3
"""SPI stress test sender for long-duration soak testing.

Sends frames continuously at a target FPS and SPI clock, logs all events,
and captures firmware serial output simultaneously.

Run on the Raspberry Pi connected to the SPI slave board.

Requirements: pip install spidev RPi.GPIO pyserial
"""
import spidev
import RPi.GPIO as GPIO
import binascii
import time
import sys
import os
import json
import argparse
import threading
import signal

READY_PIN = 16          # BCM GPIO16 on Pi
SPI_MODE = 0b11         # CPOL=1, CPHA=1 (Mode 3)

# speed sweep tiers (Hz)
SPEED_TIERS = [5_000_000, 7_000_000, 10_000_000, 12_000_000, 15_000_000]
SWEEP_INTERVAL_S = 1800  # 30 minutes per tier


def crc16_ccitt(data):
    """CRC-16-CCITT: polynomial 0x1021, init 0xFFFF. Uses C implementation."""
    return binascii.crc_hqx(data, 0xFFFF)


def build_frame_into(buf, frame_num, num_ports, pixel_data):
    """Build a framed packet into a pre-allocated bytearray. Returns length.

    Protocol: sync(2) + length(4) + frame_num(4) + flags(1) + data(N) + crc(2)
    """
    data_len = len(pixel_data)
    # header (11 bytes)
    buf[0] = 0xAA
    buf[1] = 0x55
    buf[2] = (data_len >> 24) & 0xFF
    buf[3] = (data_len >> 16) & 0xFF
    buf[4] = (data_len >> 8) & 0xFF
    buf[5] = data_len & 0xFF
    buf[6] = (frame_num >> 24) & 0xFF
    buf[7] = (frame_num >> 16) & 0xFF
    buf[8] = (frame_num >> 8) & 0xFF
    buf[9] = frame_num & 0xFF
    buf[10] = num_ports & 0x0F
    # pixel data
    buf[11:11 + data_len] = pixel_data
    # CRC over bytes 2..end (header[2:] + pixel_data)
    crc = crc16_ccitt(memoryview(buf)[2:11 + data_len])
    end = 11 + data_len
    buf[end] = (crc >> 8) & 0xFF
    buf[end + 1] = crc & 0xFF
    return end + 2


def wait_ready(timeout_s=5.0):
    """Wait for READY pin to go HIGH."""
    deadline = time.time() + timeout_s
    while not GPIO.input(READY_PIN):
        if time.time() > deadline:
            return False
        time.sleep(0.0001)
    return True


def generate_pixel_patterns(num_ports, pixels_per_port, brightness=0.3):
    """Pre-generate all pixel patterns. Returns list of (name, bytes) tuples.

    brightness: 0.0-1.0 scale applied to all RGB values.
    """
    n = num_ports * pixels_per_port * 3
    total_px = num_ports * pixels_per_port
    b = brightness

    def scale(r, g, bl):
        return [int(r * b), int(g * b), int(bl * b)]

    patterns = [
        ("red",      bytes(scale(255, 0, 0) * total_px)),
        ("green",    bytes(scale(0, 255, 0) * total_px)),
        ("blue",     bytes(scale(0, 0, 255) * total_px)),
        ("white",    bytes(scale(255, 255, 255) * total_px)),
        ("random",   bytes(int(v * b) for v in os.urandom(n))),
    ]

    # gradient
    data = bytearray(n)
    for port in range(num_ports):
        base = port * pixels_per_port * 3
        for px in range(pixels_per_port):
            v = (px * 255) // max(pixels_per_port - 1, 1)
            off = base + px * 3
            data[off] = int(v * b)
            data[off + 1] = int((255 - v) * b)
            data[off + 2] = int(((v * 3) & 0xFF) * b)
    patterns.insert(4, ("gradient", bytes(data)))

    return patterns


def serial_capture_thread(serial_port, log_file, stop_event):
    """Background thread: read firmware serial and log with timestamps."""
    try:
        import serial
        ser = serial.Serial(serial_port, 115200, timeout=1)
    except Exception as e:
        print(f"  serial: could not open {serial_port}: {e}")
        return

    try:
        with open(log_file, "w") as f:
            while not stop_event.is_set():
                try:
                    line = ser.readline()
                    if line:
                        text = line.decode("utf-8", errors="replace").rstrip()
                        ts = time.strftime("%H:%M:%S")
                        f.write(f"[{ts}] {text}\n")
                        f.flush()
                except Exception:
                    pass
    finally:
        ser.close()


def format_duration(seconds):
    """Format seconds as Xh Ym Zs."""
    h = int(seconds) // 3600
    m = (int(seconds) % 3600) // 60
    s = int(seconds) % 60
    return f"{h}h {m:02d}m {s:02d}s"


def main():
    parser = argparse.ArgumentParser(description="SPI stress test sender")
    parser.add_argument("-s", "--speed", type=int, default=15_000_000,
                        help="SPI clock Hz (default: 15000000)")
    parser.add_argument("-n", "--pixels", type=int, default=300,
                        help="pixels per port (default: 300)")
    parser.add_argument("-p", "--ports", type=int, default=8,
                        help="number of ports (default: 8)")
    parser.add_argument("-f", "--fps", type=int, default=40,
                        help="target frame rate (default: 40)")
    parser.add_argument("--duration", type=int, default=28800,
                        help="test duration seconds (default: 28800 = 8h)")
    parser.add_argument("--serial", type=str, default="/dev/ttyACM0",
                        help="firmware serial port (default: /dev/ttyACM0)")
    parser.add_argument("--log-dir", type=str, default="./stress_logs",
                        help="output directory for logs (default: ./stress_logs)")
    parser.add_argument("--speed-sweep", action="store_true",
                        help="ramp SPI speed every 30min: 5, 7, 10, 12, 15 MHz")
    parser.add_argument("--ready-pin", type=int, default=READY_PIN,
                        help="BCM GPIO pin for READY signal (default: 16)")
    args = parser.parse_args()

    ready_pin = args.ready_pin

    # create timestamped log directory
    run_name = time.strftime("%Y-%m-%d_%H%M%S")
    log_path = os.path.join(args.log_dir, run_name)
    os.makedirs(log_path, exist_ok=True)

    sender_log = os.path.join(log_path, "pi_sender.jsonl")
    serial_log = os.path.join(log_path, "firmware_serial.log")

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(ready_pin, GPIO.IN)

    spi = spidev.SpiDev()
    spi.open(0, 0)
    spi.max_speed_hz = args.speed
    spi.mode = SPI_MODE

    # start serial capture thread
    stop_serial = threading.Event()
    serial_thread = threading.Thread(
        target=serial_capture_thread,
        args=(args.serial, serial_log, stop_serial),
        daemon=True,
    )
    serial_thread.start()

    # stats
    total_frames = 0
    ready_timeouts = 0
    current_speed = args.speed
    start_time = time.time()
    last_summary_time = start_time
    sweep_start_time = start_time
    tier_index = 0

    # graceful shutdown
    shutdown = threading.Event()

    def handle_signal(signum, frame):
        shutdown.set()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    frame_period = 1.0 / args.fps
    bytes_per_frame = args.ports * args.pixels * 3

    # ensure spidev kernel buffer is large enough for a full frame in one
    # CS cycle. writebytes2 splits transfers exceeding bufsiz into separate
    # ioctl calls, which toggles CS between chunks and breaks framing.
    pkt_size = bytes_per_frame + 13
    bufsiz_path = "/sys/module/spidev/parameters/bufsiz"
    try:
        with open(bufsiz_path) as f:
            bufsiz = int(f.read().strip())
        if bufsiz < pkt_size:
            needed = max(pkt_size, 65536)
            try:
                with open(bufsiz_path, "w") as f:
                    f.write(str(needed))
                print(f"  spidev bufsiz: {bufsiz} -> {needed}")
            except PermissionError:
                print(f"ERROR: spidev bufsiz={bufsiz} < frame size {pkt_size}")
                print(f"  run: sudo sh -c 'echo {needed} > {bufsiz_path}'")
                GPIO.cleanup()
                sys.exit(1)
    except FileNotFoundError:
        pass  # non-standard kernel, hope for the best

    print(f"=== SPI STRESS TEST ===")
    print(f"SPI:      {current_speed / 1e6:.1f} MHz, Mode 3")
    print(f"Config:   {args.ports} ports x {args.pixels} px = {bytes_per_frame} bytes/frame")
    print(f"Target:   {args.fps} fps for {format_duration(args.duration)}")
    print(f"Sweep:    {'ON (' + ', '.join(f'{s/1e6:.0f}M' for s in SPEED_TIERS) + ')' if args.speed_sweep else 'OFF'}")
    print(f"Logs:     {log_path}/")
    print()

    # wait for initial READY
    print("waiting for READY...", end="", flush=True)
    if not wait_ready(timeout_s=10.0):
        print(" timeout! check wiring and firmware.")
        GPIO.cleanup()
        sys.exit(1)
    print(" ok")

    # send alignment frame
    spi.writebytes2([0x00] * 4)
    print("  sync: 4-byte alignment frame sent")
    time.sleep(0.5)

    # pre-generate all pixel patterns and a reusable frame buffer
    patterns = generate_pixel_patterns(args.ports, args.pixels)
    frame_buf = bytearray(bytes_per_frame + 13)  # header(11) + data + crc(2)
    num_patterns = len(patterns)

    print(f"  streaming at {args.fps} fps...")
    print()

    try:
        with open(sender_log, "w") as f:
            while not shutdown.is_set():
                elapsed = time.time() - start_time
                if elapsed >= args.duration:
                    break

                # speed sweep: rotate through tiers
                if args.speed_sweep:
                    sweep_elapsed = time.time() - sweep_start_time
                    new_tier = min(int(sweep_elapsed // SWEEP_INTERVAL_S),
                                   len(SPEED_TIERS) - 1)
                    if new_tier != tier_index:
                        tier_index = new_tier
                        current_speed = SPEED_TIERS[tier_index]
                        spi.max_speed_hz = current_speed
                        print(f"  [sweep] SPI speed -> {current_speed / 1e6:.0f} MHz")

                frame_start = time.time()

                # wait for READY
                ready_start = time.time()
                if not wait_ready(timeout_s=0.5):
                    ready_timeouts += 1
                    entry = {
                        "t": round(frame_start - start_time, 3),
                        "frame": total_frames + 1,
                        "event": "ready_timeout",
                        "spi_mhz": round(current_speed / 1e6, 1),
                    }
                    f.write(json.dumps(entry) + "\n")
                    continue

                ready_ms = (time.time() - ready_start) * 1000

                total_frames += 1
                _, pixel_data = patterns[total_frames % num_patterns]
                pkt_len = build_frame_into(frame_buf, total_frames,
                                           args.ports, pixel_data)
                spi.writebytes2(memoryview(frame_buf)[:pkt_len])

                # log frame
                entry = {
                    "t": round(frame_start - start_time, 3),
                    "frame": total_frames,
                    "bytes": pkt_len,
                    "ready_ms": round(ready_ms, 2),
                    "spi_mhz": round(current_speed / 1e6, 1),
                }
                f.write(json.dumps(entry) + "\n")

                # periodic console summary (every 60s)
                now = time.time()
                if now - last_summary_time >= 60:
                    run_elapsed = now - start_time
                    avg_fps = total_frames / run_elapsed if run_elapsed > 0 else 0
                    ts = format_duration(run_elapsed)
                    print(f"  [{ts}] sent:{total_frames} "
                          f"ready_timeout:{ready_timeouts} "
                          f"avg_fps:{avg_fps:.1f} "
                          f"spi:{current_speed / 1e6:.0f}MHz")
                    last_summary_time = now
                    f.flush()

                # pace to target FPS
                frame_elapsed = time.time() - frame_start
                sleep_time = frame_period - frame_elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

    finally:
        stop_serial.set()

        # final summary
        end_time = time.time()
        duration = end_time - start_time
        avg_fps = total_frames / duration if duration > 0 else 0
        total_data = total_frames * len(frame_buf)

        print()
        print(f"=== STRESS TEST COMPLETE ===")
        print(f"Duration:       {format_duration(duration)}")
        print(f"Frames sent:    {total_frames:,}")
        print(f"READY timeouts: {ready_timeouts}")
        print(f"Avg FPS:        {avg_fps:.1f}")
        print(f"SPI speed:      {current_speed / 1e6:.0f} MHz")
        print(f"Total bytes:    {total_data:,}")
        print(f"Log files:      {log_path}/")

        serial_thread.join(timeout=2)
        spi.close()
        GPIO.cleanup()


if __name__ == "__main__":
    main()
