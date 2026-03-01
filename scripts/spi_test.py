#!/usr/bin/env python3
"""Test SPI connection to SCORPIO RP2040.

Sends test frames over SPI0 to verify the SCORPIO SPI slave is receiving
and outputting to WS281x LED strips. Supports Phase 4 protocol with
CRC-16-CCITT validation.

Run on the Raspberry Pi with SCORPIO wired per the hardware plan.

Requirements: pip install spidev RPi.GPIO
"""
import spidev
import RPi.GPIO as GPIO
import binascii
import time
import sys
import argparse

READY_PIN = 16          # BCM GPIO16 on Pi (wired to GPIO10 on SCORPIO)
SPI_SPEED = 1_000_000   # 1 MHz (diagnostic: PL022 slave max = clk_peri/12 = 15+ MHz)
SPI_MODE = 0b11         # CPOL=1, CPHA=1 (Mode 3)

COLORS = [
    ([255, 0, 0],     "red"),
    ([0, 255, 0],     "green"),
    ([0, 0, 255],     "blue"),
    ([255, 255, 0],   "yellow"),
    ([255, 255, 255], "white"),
    ([0, 0, 0],       "off"),
]


def crc16_ccitt(data):
    """CRC-16-CCITT: polynomial 0x1021, init 0xFFFF. Uses C implementation."""
    return binascii.crc_hqx(bytes(data), 0xFFFF)


def wait_ready(timeout_s=5.0):
    """wait for SCORPIO READY pin to go HIGH."""
    deadline = time.time() + timeout_s
    while not GPIO.input(READY_PIN):
        if time.time() > deadline:
            return False
        time.sleep(0.001)
    return True


def build_frame(frame_num, num_ports, pixel_data):
    """build a framed packet with CRC-16.

    protocol: sync(2) + len(4) + frame_num(4) + flags(1) + data(N) + crc(2)
    flags bits 3:0 = port count
    CRC covers bytes after sync: len + frame_num + flags + data
    """
    data_len = len(pixel_data)
    header = [
        0xAA, 0x55,                         # sync word
        (data_len >> 24) & 0xFF,            # length (big-endian uint32)
        (data_len >> 16) & 0xFF,
        (data_len >> 8) & 0xFF,
        data_len & 0xFF,
        (frame_num >> 24) & 0xFF,           # frame number (big-endian)
        (frame_num >> 16) & 0xFF,
        (frame_num >> 8) & 0xFF,
        frame_num & 0xFF,
        num_ports & 0x0F,                   # flags: port count in bits 3:0
    ]

    # CRC-16 over everything after sync: header[2:] + pixel_data
    crc_data = header[2:] + list(pixel_data)
    crc = crc16_ccitt(crc_data)

    return header + list(pixel_data) + [(crc >> 8) & 0xFF, crc & 0xFF]


def main():
    parser = argparse.ArgumentParser(description="SCORPIO SPI test")
    parser.add_argument("-n", "--pixels", type=int, default=100,
                        help="pixels per port (default: 100)")
    parser.add_argument("-p", "--ports", type=int, default=1,
                        help="number of ports (default: 1)")
    parser.add_argument("-s", "--speed", type=int, default=SPI_SPEED,
                        help="SPI speed in Hz (default: 1000000)")
    parser.add_argument("-d", "--delay", type=float, default=2.0,
                        help="seconds between frames (default: 2.0)")
    args = parser.parse_args()

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(READY_PIN, GPIO.IN)

    spi = spidev.SpiDev()
    spi.open(0, 0)
    spi.max_speed_hz = args.speed
    spi.mode = SPI_MODE

    # ensure spidev kernel buffer fits a full frame in one CS cycle
    pkt_size = args.ports * args.pixels * 3 + 13
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
                spi.close()
                GPIO.cleanup()
                sys.exit(1)
    except FileNotFoundError:
        pass

    try:
        print(f"SPI: {args.speed / 1e6:.1f} MHz, mode {SPI_MODE}")
        print(f"config: {args.ports} port(s), {args.pixels} pixels/port")
        print("waiting for READY...", end="", flush=True)
        if not wait_ready():
            print(" timeout! check wiring and firmware.")
            sys.exit(1)
        print(" ok")

        # send a dummy frame to align the PL022 shift register.
        # CPHA=1 causes a 1-bit shift on the very first transfer after boot.
        spi.writebytes2([0x00] * 4)
        print("  sync: 4-byte alignment frame sent")
        time.sleep(0.5)

        for i, (rgb, name) in enumerate(COLORS):
            if not wait_ready(timeout_s=2.0):
                print(f"  frame {i + 1}: READY timeout, stopping")
                break

            # build port-sequential pixel data: all pixels for port0, then port1, etc.
            port_pixels = rgb * args.pixels
            pixel_data = port_pixels * args.ports

            packet = build_frame(i + 1, args.ports, pixel_data)
            spi.writebytes2(packet)

            data_bytes = len(pixel_data)
            print(f"  frame {i + 1}: {name:6s} ({rgb[0]:3d},{rgb[1]:3d},{rgb[2]:3d}) "
                  f"- {args.pixels}px x {args.ports}port "
                  f"- {len(packet)} bytes")
            time.sleep(args.delay)

        print("done")
    finally:
        spi.close()
        GPIO.cleanup()


if __name__ == "__main__":
    main()
