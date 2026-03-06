#!/usr/bin/env python3
"""Test SPI connection to RP2040/RP2350 LED boards.

Sends test frames over SPI0 to verify the SPI slave is receiving
and outputting to WS281x LED strips. Uses protocol v2 with per-port
bitmask, CRC-16-CCITT validation, and optional DMX.

Run on the Raspberry Pi with the RP2xxx board wired per the hardware plan.

Requirements: pip install spidev RPi.GPIO
"""
import spidev
import RPi.GPIO as GPIO
import binascii
import time
import sys
import argparse

READY_PIN = 16          # BCM GPIO16 on Pi (wired to READY on RP2xxx board)
SPI_SPEED = 1_000_000   # 1 MHz (diagnostic: PL022 slave max = clk_peri/12)
SPI_MODE = 0b11         # CPOL=1, CPHA=1 (Mode 3)

PROTOCOL_VERSION = 2
FLAG_DMX = 0x01

COLORS = [
    ([255, 0, 0],     "red"),
    ([0, 255, 0],     "green"),
    ([0, 0, 255],     "blue"),
    ([255, 255, 0],   "yellow"),
    ([255, 255, 255], "white"),
    ([0, 0, 0],       "off"),
]


def crc16_ccitt(data):
    """CRC-16-CCITT: polynomial 0x1021, init 0xFFFF."""
    return binascii.crc_hqx(bytes(data), 0xFFFF)


def wait_ready(timeout_s=5.0):
    """wait for READY pin to go HIGH."""
    deadline = time.time() + timeout_s
    while not GPIO.input(READY_PIN):
        if time.time() > deadline:
            return False
        time.sleep(0.001)
    return True


def build_frame(frame_num, port_mask, pixel_data, dmx_data=None):
    """build a v2 framed packet with CRC-16.

    protocol v2: sync(2) + len(4) + frame_num(4) + flags(1) + port_mask(4) + data(N) + crc(2)
    flags: bits 7:4 = protocol version (2), bit 0 = DMX data follows
    CRC covers bytes 2 through end-2 (everything between sync and CRC)
    """
    flags = (PROTOCOL_VERSION << 4)

    payload = list(pixel_data)
    if dmx_data is not None:
        flags |= FLAG_DMX
        dmx_bytes = list(dmx_data)
        dmx_len = len(dmx_bytes)
        payload += dmx_bytes
        payload += [(dmx_len >> 8) & 0xFF, dmx_len & 0xFF]

    data_len = len(payload)
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
        flags,                              # version(4) | dmx(1)
        (port_mask >> 24) & 0xFF,           # port mask (big-endian uint32)
        (port_mask >> 16) & 0xFF,
        (port_mask >> 8) & 0xFF,
        port_mask & 0xFF,
    ]

    # CRC-16 over everything after sync: header[2:] + payload
    crc_data = header[2:] + payload
    crc = crc16_ccitt(crc_data)

    return header + payload + [(crc >> 8) & 0xFF, crc & 0xFF]


def popcount(n):
    """count set bits in an integer."""
    count = 0
    while n:
        count += 1
        n &= n - 1
    return count


def main():
    parser = argparse.ArgumentParser(description="RP2xxx SPI test (protocol v2)")
    parser.add_argument("-n", "--pixels", type=int, default=100,
                        help="pixels per port (default: 100)")
    parser.add_argument("-p", "--ports", type=int, default=1,
                        help="number of ports (default: 1)")
    parser.add_argument("-m", "--mask", type=str, default=None,
                        help="port mask as hex (e.g. 0x05 for ports 0+2). "
                             "overrides --ports")
    parser.add_argument("-s", "--speed", type=int, default=SPI_SPEED,
                        help="SPI speed in Hz (default: 1000000)")
    parser.add_argument("-d", "--delay", type=float, default=2.0,
                        help="seconds between frames (default: 2.0)")
    parser.add_argument("--spi-bus", type=int, default=0,
                        help="SPI bus number (default: 0)")
    parser.add_argument("--spi-cs", type=int, default=0,
                        help="SPI chip select (default: 0)")
    parser.add_argument("--ready-pin", type=int, default=READY_PIN,
                        help="BCM GPIO pin for READY signal (default: 16)")
    args = parser.parse_args()

    # determine port mask
    if args.mask is not None:
        port_mask = int(args.mask, 0)
        active_ports = popcount(port_mask)
    else:
        # consecutive ports starting from 0
        port_mask = (1 << args.ports) - 1
        active_ports = args.ports

    if port_mask == 0:
        print("ERROR: port mask is zero (no ports selected)")
        sys.exit(1)

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(args.ready_pin, GPIO.IN)

    spi = spidev.SpiDev()
    spi.open(args.spi_bus, args.spi_cs)
    spi.max_speed_hz = args.speed
    spi.mode = SPI_MODE

    # ensure spidev kernel buffer fits a full frame in one CS cycle
    # v2 header: 15 bytes, CRC: 2 bytes, pixel data: active_ports * pixels * 3
    pkt_size = active_ports * args.pixels * 3 + 17
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
        print(f"SPI: spi{args.spi_bus}.{args.spi_cs} @ {args.speed / 1e6:.1f} MHz, mode {SPI_MODE}")
        print(f"protocol v{PROTOCOL_VERSION}, port mask 0x{port_mask:08X} ({active_ports} active)")
        print(f"config: {args.pixels} pixels/port, {active_ports * args.pixels * 3} data bytes/frame")
        print(f"READY pin: GPIO{args.ready_pin}")
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

            # build pixel data for active ports only (lowest set bit first)
            port_pixels = rgb * args.pixels
            pixel_data = port_pixels * active_ports

            packet = build_frame(i + 1, port_mask, pixel_data)
            spi.writebytes2(packet)

            print(f"  frame {i + 1}: {name:6s} ({rgb[0]:3d},{rgb[1]:3d},{rgb[2]:3d}) "
                  f"- {args.pixels}px x {active_ports}port "
                  f"- {len(packet)} bytes")
            time.sleep(args.delay)

        print("done")
    finally:
        spi.close()
        GPIO.cleanup()


if __name__ == "__main__":
    main()
