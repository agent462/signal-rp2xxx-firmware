# Signal RP2040 SCORPIO Firmware

Firmware for the RP2040 board that receives LED frame data over SPI and outputs to up to 8 WS281x LED strips in parallel via PIO. Adafruit Scorpio is a great example board to implement with - wiring example below.

## How It Works

The firmware runs a dual-core pipeline on the RP2040:

- **Core 0** receives framed pixel data over SPI1 (slave mode) using DMA with double-buffered reception. A GPIO interrupt on the CS pin detects frame boundaries.
- **Core 1** transforms pixel data into bitplane format and drives all 8 LED strips simultaneously through a PIO state machine with DMA.

The cores communicate through the RP2040's hardware multicore FIFO. Core 0 validates each frame (sync word, CRC-16, port/pixel alignment) and passes a pointer to Core 1. Core 1 signals completion so the SPI buffer can be re-armed for the next frame.

The firmware is color-order-agnostic -- it outputs bytes in whatever order it receives them. Color reordering (RGB for WS2811, GRB for WS2812, etc.) should be handled upstream.

## Hardware Wiring

### Pi to SCORPIO

```
Raspberry Pi 5 (SPI0 master)          SCORPIO (SPI1 slave)
================================       ================================
Pin 19  GPIO10  SPI0_MOSI  --------->  GPIO08  MI   SPI1_RX   (slave data in)
Pin 21  GPIO09  SPI0_MISO  <---------  GPIO15  MO   SPI1_TX   (slave data out)
Pin 23  GPIO11  SPI0_SCLK  --------->  GPIO14  SCK  SPI1_SCK  (clock)
Pin 24  GPIO08  SPI0_CE0   --------->  GPIO09  D9   SPI1_CSn  (chip select)
Pin 36  GPIO16             <---------  GPIO10  D10             (READY signal)
Pin 39  GND                --------->  GND                     (common ground)
```

MO/MI labels on the SCORPIO are from the **master** perspective (Adafruit convention). In slave mode, MO = TX (slave output) and MI = RX (slave input). Pi MOSI carries master-out data, so it connects to the slave's RX pin (MI/GPIO8).

### Power

- SCORPIO: USB-C (separate from Pi)
- LED strips: separate 5V supply, common ground with SCORPIO
- Keep SPI wires under 15 cm

### SCORPIO GPIO Allocation

| GPIO | Function | Notes |
|------|----------|-------|
| 4 | Onboard NeoPixel | Status LED (RGB) |
| 8 | SPI1 RX (MI) | Slave data input from Pi MOSI |
| 9 | SPI1 CSn | Chip select from Pi |
| 10 | READY output | HIGH = ready for data, LOW = processing |
| 13 | Built-in red LED | Debug/heartbeat |
| 14 | SPI1 SCK | Clock from Pi |
| 15 | SPI1 TX (MO) | Slave data output to Pi MISO |
| 16-23 | NEOPIXEL 0-7 | WS281x outputs (level-shifted to 5V) |

## Building

### Prerequisites

**macOS (cross-compile):**

```bash
brew install cmake
brew install --cask gcc-arm-embedded   # must use the cask, not bare arm-none-eabi-gcc
cd /path/to
git clone https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk && git checkout 2.2.0 && git submodule update --init

# add to ~/.zshrc:
export PICO_SDK_PATH=/path/to/pico-sdk
```

**Raspberry Pi:**

```bash
sudo apt update
sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential
cd /home/signal
git clone https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk && git checkout 2.2.0 && git submodule update --init
export PICO_SDK_PATH=/home/signal/pico-sdk
```

The bare Homebrew `arm-none-eabi-gcc` package does not include newlib and will fail with `nosys.specs not found`. The `gcc-arm-embedded` cask bundles the full ARM toolchain.

### Build

```bash
cd firmware/rp2040
cp $PICO_SDK_PATH/external/pico_sdk_import.cmake .   # first time only
mkdir -p build && cd build
cmake ..
make -j4
```

Output: `build/signal_rp2040.uf2` (~72K)

The first `cmake` run downloads picotool 2.2.0 automatically.

## Flashing

### Using the Flash Script (recommended)

The `scorpio-flash.sh` script handles building, copying to the Pi over SSH, mounting the BOOTSEL drive, flashing, and reading serial output:

```bash
./scripts/diag/scorpio-flash.sh -b                     # build + flash
./scripts/diag/scorpio-flash.sh                         # flash pre-built UF2
./scripts/diag/scorpio-flash.sh -S                      # serial monitor only
./scripts/diag/scorpio-flash.sh -p user@10.0.0.5 -b    # custom Pi host
./scripts/diag/scorpio-flash.sh -b -s 20                # build, flash, 20s serial
```

The SCORPIO must be in BOOTSEL mode before flashing (hold BOOTSEL while plugging USB-C, or hold BOOTSEL + press RESET).

### Manual Flash

1. Hold BOOTSEL while plugging in USB-C. The board appears as a `RPI-RP2` drive.
2. Copy the UF2:
   ```bash
   # macOS (direct USB):
   cp build/signal_rp2040.uf2 /Volumes/RPI-RP2/

   # Pi (BOOTSEL drive may need manual mount on Pi OS Lite):
   sudo mkdir -p /media/signal/RPI-RP2
   sudo mount /dev/sda1 /media/signal/RPI-RP2
   cp build/signal_rp2040.uf2 /media/signal/RPI-RP2/
   ```
3. The board reboots automatically after the copy completes.

### Verify

USB serial appears at `/dev/ttyACM0` (Pi) or `/dev/cu.usbmodem*` (macOS) after ~3 seconds:

```bash
screen /dev/ttyACM0 115200        # Pi
screen /dev/cu.usbmodem* 115200   # macOS
```

You should see frame reception statistics and status messages.

## SPI Frame Protocol

Each frame sent from the Pi to the SCORPIO follows this binary format:

```
Offset  Size  Field
------  ----  -----
0       2     Sync word: 0xAA 0x55
2       2     Data length (big-endian) -- pixel data bytes only
4       4     Frame number (big-endian) -- monotonic counter
8       1     Flags: bits 3:0 = port count (1-8), bits 7:4 reserved
9       N     Pixel data (port-sequential)
9+N     2     CRC-16-CCITT (big-endian)
```

**Pixel data layout:** port-sequential. All pixels for port 0 first, then port 1, etc. Each pixel is 3 bytes in whatever color order the daemon sends (typically RGB or GRB depending on LED type).

**CRC-16-CCITT:** polynomial 0x1021, init 0xFFFF. Covers bytes 2 through 8+N (everything after the sync word, excluding the CRC itself).

**Validation:** The firmware checks sync word, data length bounds, port count range (1-8), pixel data alignment (divisible by ports x 3), and CRC match. Bad frames are dropped and the last good frame continues displaying.

### Example

10 pixels on 1 port (30 bytes of RGB data):

```
AA 55              sync
00 1E              length = 30
00 00 00 01        frame number = 1
01                 flags = 1 port
[30 bytes RGB]     pixel data
XX XX              CRC-16
```

## Testing with the Diagnostic Script

A Python test script sends color test frames from the Pi:

```bash
# basic test: 1 port, 100 pixels, 1 MHz SPI
python3 scripts/diag/spi_test.py

# multi-port test
python3 scripts/diag/spi_test.py -n 300 -p 4

# full speed test
python3 scripts/diag/spi_test.py -n 300 -p 8 -s 7000000

# custom delay between frames
python3 scripts/diag/spi_test.py -n 100 -p 2 -d 1.0
```

Requirements on the Pi: `pip install spidev RPi.GPIO`

SPI must be enabled: `sudo raspi-config` -> Interface Options -> SPI -> Enable

The script sends a 4-byte dummy frame first to align the PL022 shift register (CPHA=1 quirk), then cycles through red, green, blue, yellow, white, and off.

## Performance

```
Full capacity:   8 ports x 800 pixels x 3 bytes = 19,200 bytes/frame
With overhead:   19,200 + 11 header/CRC = 19,211 bytes/frame
SPI at 7 MHz:    875,000 bytes/sec
Max fps:         ~45 fps at full capacity
```

The PL022 slave maximum SPI clock is 7.5 MHz (system clock 180 MHz, prescaler 2, 12x oversampling: 90 / 12 = 7.5 MHz). At typical pixel counts (300 pixels, 4 ports), frame rates well above 40 fps are achievable.

## Architecture Details

### SPI Slave (PL022)

The RP2040's PL022 SPI peripheral runs in slave mode with several constraints that the firmware works around:

- **Mode 3 required (CPOL=1, CPHA=1):** In CPHA=0 modes, the PL022 slave requires CS to toggle between every byte, limiting transfers to 1 byte at a time. Mode 3 allows continuous multi-byte transfers.
- **Bypasses `spi_init()`:** The SDK's `spi_init()` sets a slow prescaler and briefly enables the peripheral, which violates slave oversampling requirements. The firmware uses `reset_block()`/`unreset_block_wait()` and configures registers directly.
- **SCK pull-up:** CPOL=1 means SCK idles HIGH. A pull-up on the SCK pin prevents false clock edges when the master is disconnected.
- **DREQ overrun:** After the last real byte, the PL022 DREQ stays asserted and DMA reads zeros at bus speed, filling the entire buffer. Frame length comes from the protocol header, not the DMA transfer count.
- **First-frame alignment:** CPHA=1 causes a 1-bit shift on the very first transfer after boot. A 4-byte dummy frame aligns the shift register.

### PIO WS281x Output

A 4-instruction PIO program drives all 8 strips in parallel:

1. Pull 32 bits from TX FIFO (1 bit per strip per bit-position)
2. Set all pins HIGH (375 ns)
3. Set pins to data value (375 ns) -- 0-bits go LOW, 1-bits stay HIGH
4. Set all pins LOW (375 ns)

At 180 MHz system clock with divider 22.5, the PIO runs at 8 MHz (10 cycles per WS281x bit = 800 kHz bit rate). DMA feeds the PIO TX FIFO from the bitplane buffer without CPU involvement.

### Bitplane Transform

Pixel data arrives port-sequential (all port 0 pixels, then port 1, etc.) but the PIO needs bitplane format (1 bit per port per bit-position). The bitplane transform on Core 1 converts between these formats:

- Input: `port0_pix0_R, port0_pix0_G, port0_pix0_B, port0_pix1_R, ...`
- Output: 32-bit words where bit N is set if port N has that bit HIGH for that pixel/color/bit position
- Total output: `pixels_per_port * 24` words (24 bits per RGB pixel)

## File Overview

| File | Purpose |
|------|---------|
| `CMakeLists.txt` | Build configuration (pico-sdk 2.2.0, USB serial) |
| `pico_sdk_import.cmake` | SDK bootstrapper (copied from pico-sdk) |
| `src/config.h` | Pin definitions, clock speed, protocol constants |
| `src/main.c` | Dual-core entry point and frame processing loop |
| `src/spi_slave.h/.c` | SPI1 slave with DMA reception and READY pin |
| `src/frame_protocol.h/.c` | Frame parsing, validation, CRC-16-CCITT |
| `src/ws281x_parallel.h/.c` | PIO init and DMA-driven parallel LED output |
| `src/ws281x_parallel.pio` | PIO assembly for WS281x bit timing |
| `src/bitplane.h/.c` | Port-sequential to bitplane format conversion |

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| `nosys.specs not found` | Wrong ARM toolchain | Use `brew install --cask gcc-arm-embedded`, not `brew install arm-none-eabi-gcc` |
| `PICO_SDK_PATH not set` | Environment variable missing | `export PICO_SDK_PATH=/path/to/pico-sdk` and re-run cmake |
| No RPI-RP2 drive | Not in BOOTSEL mode | Hold BOOTSEL while plugging USB-C, or hold BOOTSEL + press RESET |
| No serial output | USB stdio not enabled | Ensure `pico_enable_stdio_usb(signal_rp2040 1)` in CMakeLists.txt |
| No SPI data received | Wiring swapped | Pi MOSI (GPIO10) goes to SCORPIO MI/GPIO8 (SPI1_RX). Labels are master-perspective. |
| Garbage SPI data | Mode mismatch | Both sides must be Mode 3 (CPOL=1, CPHA=1). First frame after boot needs 4-byte dummy. |
| SPI works once then stops | READY not re-armed | Check that `spi_slave_frame_consumed()` is called after each frame |
| No LEDs | Wrong pin base | LED outputs are GPIO 16-23, not GPIO 0-7 |
| Wrong LED colors | Color order mismatch | Set `colorOrder` per virtual string in daemon config (RGB for WS2811, GRB for WS2812) |
| Flickering LEDs | PIO clock wrong | Clock divider must be 22.5 at 180 MHz (180 / 8 MHz = 22.5) |
| BOOTSEL not mounting on Pi | Pi OS Lite has no automounter | Use `scorpio-flash.sh` or mount manually with `sudo mount /dev/sda1` |
