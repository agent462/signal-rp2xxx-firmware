# Signal SPI Output Firmware

Firmware for RP2040/RP2350 microcontrollers that receive LED frame data daemon over SPI and drive WS281x LED strips via PIO.

## Supported Boards

| Board | MCU | Channels | Level Shifter | Config |
|-------|-----|----------|---------------|--------|
| SCORPIO | Adafruit Feather RP2040 SCORPIO | 8 | Built-in | `boards/scorpio/` |
| Pico 2 W 8-ch | Raspberry Pi Pico 2 W (RP2350) | 8 | External SN74AHCT245N | `boards/pico2w_8ch/` |
| Signal 8 | RP2350A on custom carrier (CM5) | 8 native + 4 differential + DMX | 74HCT245 + RS-485 | `boards/signal8/` |

## Directory Structure

```
common/                     shared source files (all boards)
  main.c                    dual-core main loop (core0: SPI, core1: PIO)
  spi_slave.h / .c          PL022 SPI slave with DMA reception
  frame_protocol.h / .c     frame parsing, CRC-16 validation
  bitplane.h / .c           pixel-to-bitplane transform for PIO
  ws281x_parallel.pio       PIO program for parallel WS281x output
  ws281x_parallel.h / .c    PIO init and DMA output (multi-instance)
  dmx_output.pio             PIO UART TX for DMX512 (250kbaud 8N2)
  dmx_output.h / .c          DMX init, BREAK/MAB, PIO byte output
  status_led.h / .c         onboard NeoPixel status (SCORPIO only)
  pico_sdk_import.cmake     Pico SDK locator
boards/
  scorpio/                  Adafruit Feather RP2040 SCORPIO
    config.h                pin assignments, SPI1 on GPIO8/9/14/15
    CMakeLists.txt          PICO_BOARD defaults to pico
  pico2w_8ch/               Pico 2 W + SN74AHCT245N (8 channels)
    config.h                pin assignments, SPI0 on GPIO16-19
    CMakeLists.txt          PICO_BOARD=pico2_w
  signal8/                  Signal 8 controller (RP2350A + CM5 carrier)
    config.h                pin assignments, 8 native + 4 diff + DMX
    CMakeLists.txt          PICO_BOARD=pico2
scripts/diag/
  firmware-flash.sh         build, flash via BOOTSEL, monitor serial
  spi_test.py               send test frames from Pi to verify SPI
```

Each board has only a `config.h` (pin assignments) and `CMakeLists.txt`. All firmware logic is shared.

## Building

### Prerequisites

```bash
# macOS
brew install cmake
brew install --cask gcc-arm-embedded

# Pico SDK (2.2.0+ required for Pico 2 W)
cd ~/Github
git clone https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk && git checkout 2.2.0 && git submodule update --init
export PICO_SDK_PATH=~/Github/pico-sdk
```

### Build Commands

```bash
# SCORPIO
cd boards/scorpio
mkdir -p build && cd build
cmake .. && make -j4
# output: build/signal_rp2040.uf2

# Pico 2 W (8-channel)
cd boards/pico2w_8ch
mkdir -p build && cd build
cmake .. && make -j4
# output: build/signal_pico2w.uf2

# Signal 8 (8 native + 4 differential)
cd boards/signal8
mkdir -p build && cd build
cmake .. && make -j4
# output: build/signal_8.uf2
```

### Flash

Hold BOOTSEL while plugging USB. Copy the UF2 to the mounted drive:

```bash
# macOS (board appears as RPI-RP2 volume)
cp build/signal_pico2w.uf2 /Volumes/RPI-RP2/

# Pi (manual mount)
sudo mount /dev/sda1 /media/$USER/RPI-RP2
sudo cp build/signal_pico2w.uf2 /media/$USER/RPI-RP2/
```

Or use the flash script from a Mac to build, SCP, and flash on a remote Pi:

```bash
./scripts/diag/firmware-flash.sh --board scorpio -b       # SCORPIO (default)
./scripts/diag/firmware-flash.sh --board pico2w_8ch -b    # Pico 2 W
./scripts/diag/firmware-flash.sh -S                       # serial monitor only
```

### Monitor Serial

```bash
screen /dev/cu.usbmodem* 115200   # macOS
screen /dev/ttyACM0 115200        # Pi
```

## Hardware Wiring

### Pi to SCORPIO

```
Raspberry Pi 5 (SPI0 master)          SCORPIO (SPI1 slave)
================================       ================================
Pin 19  GPIO10  SPI0_MOSI  --------->  GPIO08  MI   SPI1_RX  (slave input)
Pin 21  GPIO09  SPI0_MISO  <---------  GPIO15  MO   SPI1_TX  (slave output)
Pin 23  GPIO11  SPI0_SCLK  --------->  GPIO14  SCK  SPI1_SCK (clock)
Pin 24  GPIO08  SPI0_CE0   --------->  GPIO09  D9   SPI1_CSn (chip select)
Pin 36  GPIO16             <---------  GPIO10  D10            (READY signal)
Pin 39  GND                --------->  GND                    (common ground)

Power: USB-C to SCORPIO.

NOTE: Adafruit labels MO/MI from the master perspective. In slave mode,
MO = TX (slave output to Pi MISO) and MI = RX (slave input from Pi MOSI).
```

### Pi to Pico 2 W

```
Raspberry Pi 5 (SPI0 master)          Pico 2 W (SPI0 slave)
================================       ================================
Pin 19  GPIO10  SPI0_MOSI  --------->  GPIO16  SPI0_RX   (slave data in)
Pin 21  GPIO09  SPI0_MISO  <---------  GPIO19  SPI0_TX   (slave data out)
Pin 23  GPIO11  SPI0_SCLK  --------->  GPIO18  SPI0_SCK  (clock)
Pin 24  GPIO08  SPI0_CE0   --------->  GPIO17  SPI0_CSn  (chip select)
Pin 36  GPIO16             <---------  GPIO20             (READY signal)
Pin 39  GND                --------->  GND                (common ground)

Power: USB-C to Pico 2 W (or 5V from Pi VSYS to Pico VSYS pin).
```

### Pico 2 W Level Shifter + Pixels

```
Pico 2 W                  SN74AHCT245N              Pixels
==========                ==============             =========
GPIO0 (3.3V) -----------> A1 (pin 2)
GPIO1 (3.3V) -----------> A2 (pin 3)
GPIO2 (3.3V) -----------> A3 (pin 4)     B1 (pin 18) --[330R]--> Ch1 WS281x
GPIO3 (3.3V) -----------> A4 (pin 5)     B2 (pin 17) --[330R]--> Ch2 WS281x
GPIO4 (3.3V) -----------> A5 (pin 6)     B3 (pin 16) --[330R]--> Ch3 WS281x
GPIO5 (3.3V) -----------> A6 (pin 7)     B4 (pin 15) --[330R]--> Ch4 WS281x
GPIO6 (3.3V) -----------> A7 (pin 8)     B5 (pin 14) --[330R]--> Ch5 WS281x
GPIO7 (3.3V) -----------> A8 (pin 9)     B6 (pin 13) --[330R]--> Ch6 WS281x
                                          B7 (pin 12) --[330R]--> Ch7 WS281x
                                          B8 (pin 11) --[330R]--> Ch8 WS281x
                           VCC (pin 20) = 5V
                           GND (pin 10) = common ground
                           DIR (pin 1)  = tied to VCC (A-to-B)
                           OE  (pin 19) = tied to GND (always enabled)

Decoupling: 100nF ceramic + 10uF electrolytic on '245 VCC, close to pin 20.
B-side pins are in REVERSE order (B1=pin 18, B8=pin 11).
```

## SPI Protocol

A daemon sends frames over SPI at up to 7 MHz. The firmware is a SPI slave using PL022 Mode 3 (CPOL=1, CPHA=1).

### Frame Format

```
Offset  Size  Field
------  ----  -----
0       2     Sync word: 0xAA55
2       4     Data length (big-endian uint32, total payload bytes)
6       4     Frame number (big-endian uint32, monotonic counter)
10      1     Flags: bits 3:0 = port count (1-15), bit 4 = DMX data follows
11      N     Pixel data (port-sequential: all port0 pixels, then port1, ...)
              If flags bit 4 set:
11+P    D       DMX channel data (D bytes, 1-512)
11+P+D  2       DMX data length (big-endian, D = 1-512)
end-1   2     CRC-16-CCITT (big-endian, over bytes 2 through end-2)
```

CRC-16-CCITT: polynomial 0x1021, init 0xFFFF. Covers LENGTH + FRAME_NUM + FLAGS + all payload DATA (pixel + DMX).

When bit 4 of flags is clear (standard frame), the data length equals pixel data length and the format is backward-compatible with existing senders. When bit 4 is set, pixel data occupies the first `ports * pixels_per_port * 3` bytes, followed by the DMX channel data and a 2-byte DMX data length at the end of the payload.

### Flow Control

The READY pin implements hardware flow control:
- **HIGH**: firmware is ready to receive a frame
- **LOW**: firmware is processing (Pi must wait before sending)

### Performance

```
SPI slave max clock = f_SSPCLK / 12 (PL022 requirement)
  RP2040  @ 180 MHz: CPSR=2 -> 90 MHz / 12 = 7.5 MHz (0.94 MB/s)
  RP2350  @ 250 MHz: CPSR=2 -> 125 MHz / 12 = 10.42 MHz (1.30 MB/s)

WS281x output time = pixels × 24 bits × 1.25µs
  300 pixels: 9ms   (max ~111 fps)
  800 pixels: 24ms  (max ~41 fps)

8 ports x 300 pixels x 3 bytes = 7,200 bytes/frame
  SPI: 7.7ms @ 7.5 MHz  -> WS281x-limited @ 111 fps

8 ports x 800 pixels x 3 bytes = 19,200 bytes/frame
  SPI: 20.5ms @ 7.5 MHz -> WS281x-limited @ 41 fps

Signal 8 (12 ports, RP2350 @ 250 MHz):
12 ports x 300 pixels x 3 bytes = 10,800 bytes/frame
  SPI: 8.3ms @ 10.42 MHz -> WS281x-limited @ 111 fps

12 ports x 800 pixels x 3 bytes = 28,800 bytes/frame
  SPI: 22.1ms @ 10.42 MHz -> WS281x-limited @ 41 fps
```

### SPI Slave Notes

- PL022 slave max SCK = 7.5 MHz (f_SSPCLK = 180 MHz / CPSR=2 = 90 MHz, /12 = 7.5 MHz)
- Mode 3 (CPHA=1) required for multi-byte transfers with CS held low
- First frame after boot is bit-shifted; daemon sends a 4-byte dummy to align
- SCK needs pull-up (CPOL=1 idles HIGH)
- PL022 DREQ overrun: firmware disables RXDMAE on CS rising edge to prevent DMA racing through buffer with zero reads

## Architecture

```
Core 0                          Core 1
------                          ------
SPI slave DMA reception    -->  Bitplane transform (native + differential)
Frame protocol parsing     -->  PIO DMA output to LEDs
READY pin management            DMX512 PIO UART output (Signal 8 only)
Watchdog + timeout blanking
```

Core 0 receives SPI frames via DMA, validates (sync, CRC, alignment), and passes a frame dispatch struct to Core 1 via multicore FIFO. Core 1 transforms port-sequential pixel bytes into bitplane format and feeds the PIO state machine(s) via DMA. On Signal 8, Core 1 also drives DMX512 output via a PIO-based UART.

The firmware is color-order-agnostic. It outputs bytes in whatever order it receives them. Color order (RGB for WS2811, GRB for WS2812, etc.) is handled upstream.

### Signal 8 PIO Allocation

| PIO Block | SM | Function | GPIOs |
|-----------|-----|----------|-------|
| pio0 | SM0 | Native WS281x (8 ch) | GPIO8-15 |
| pio1 | SM0 | Differential WS281x (4 ch) | GPIO16-19 |
| pio2 | SM0 | DMX512 UART TX | GPIO21 |

### CM5 to Signal 8

```
CM5 (SPI0 master)                Signal 8 RP2350A (SPI0 slave)
================================  ================================
SPI0_MOSI  ------------------>  GPIO0   SPI0_RX   (slave data in)
SPI0_CE0   ------------------>  GPIO1   SPI0_CSn  (chip select)
SPI0_SCLK  ------------------>  GPIO2   SPI0_SCK  (clock)
SPI0_MISO  <------------------  GPIO3   SPI0_TX   (slave data out)
GPIO (flow ctl)  <------------  GPIO4              (READY signal)
                                GPIO5              (status LED)

Native pixel outputs (via 74HCT245 level shifter):
GPIO8  --> 74HCT245 --> Ch1 WS281x (Phoenix connector)
GPIO9  --> 74HCT245 --> Ch2 WS281x
...
GPIO15 --> 74HCT245 --> Ch8 WS281x

Differential pixel outputs (via RS-485 drivers):
GPIO16 --> RS-485 TX --> RJ45 Ch1 (pins 1/2)
GPIO17 --> RS-485 TX --> RJ45 Ch2 (pins 3/6)
GPIO18 --> RS-485 TX --> RJ45 Ch3 (pins 4/5)
GPIO19 --> RS-485 TX --> RJ45 Ch4 (pins 7/8)
GPIO20 --> RS-485 DE (direction: shared for ch 1-4)

DMX512 output (via RS-485 driver):
GPIO21 --> RS-485 TX --> RJ45 DMX
GPIO22 --> RS-485 DE (direction: DMX)
```

## Adding a New Board

1. Create `boards/<name>/config.h` with pin assignments (see existing configs)
2. Create `boards/<name>/CMakeLists.txt` (copy from an existing board, change project name and `PICO_BOARD`)
3. Add a case to `scripts/diag/firmware-flash.sh` for the UF2 filename mapping

Required defines in `config.h`:

| Define | Purpose |
|--------|---------|
| `BOARD_NAME` | String for boot message |
| `SYS_CLOCK_KHZ` | System clock (180000 for 7.5 MHz SPI slave) |
| `NEO_PIN_BASE` | First GPIO for PIO LED outputs |
| `NUM_PORTS` | Number of parallel LED outputs |
| `MAX_PIXELS` | Max pixels per port |
| `SPI_PORT` | `spi0` or `spi1` |
| `SPI_RESET_BITS` | `RESETS_RESET_SPI0_BITS` or `RESETS_RESET_SPI1_BITS` |
| `PIN_SPI_RX/TX/SCK/CS` | SPI GPIO pins |
| `PIN_READY` | READY signal GPIO |
| `MAX_FRAME_SIZE` | `(NUM_PORTS * MAX_PIXELS * 3)` |

Optional:

| Define | Purpose |
|--------|---------|
| `PIN_LED` | Heartbeat toggle LED |
| `PIN_NEOPIXEL` | Color status indicator (NeoPixel) |
| `NUM_DIFF_PORTS` | Differential output channel count (enables dual-PIO) |
| `DIFF_PIN_BASE` | First GPIO for differential PIO outputs |
| `PIN_DIFF_DIR` | RS-485 direction pin for differential channels |
| `TOTAL_PORTS` | Total channels (native + differential), extends port validation |
| `PIN_DMX_TX` | DMX512 PIO UART TX pin (enables DMX output) |
| `PIN_DMX_DIR` | RS-485 direction pin for DMX |
