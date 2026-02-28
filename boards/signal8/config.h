#ifndef SIGNAL_BOARD_CONFIG_H
#define SIGNAL_BOARD_CONFIG_H

// board identity
#define BOARD_NAME "signal8"

// system clock: 250 MHz (RP2350 stable max).
// PL022 slave requires f_SSPCLK >= f_SCK/12; at 250 MHz with CPSR=2,
// f_SSPCLK = 125 MHz -> max SPI slave clock = 10.42 MHz.
#define SYS_CLOCK_KHZ 250000

// ---------------------------------------------------------------------------
// Native pixel outputs (8x Phoenix connectors via 74HCT245 level shifter)
// GPIO8-15 -> 74HCT245 A-side -> B-side (5V) -> Phoenix connectors
// Contiguous block for PIO parallel output efficiency.
// ---------------------------------------------------------------------------
#define NEO_PIN_BASE 8
#define NUM_PORTS    8
#define MAX_PIXELS   800

// ---------------------------------------------------------------------------
// Differential pixel outputs (4 channels via RS-485 drivers -> RJ45)
// GPIO16-19 -> RS-485 TX -> RJ45 twisted pairs (T568B pinout)
// Falcon/Kulp-compatible: Ch1 pins 1/2, Ch2 3/6, Ch3 4/5, Ch4 7/8
// Driven by pio1/SM0 (separate PIO instance from native outputs).
// ---------------------------------------------------------------------------
#define DIFF_PIN_BASE    16
#define NUM_DIFF_PORTS   4
#define PIN_DIFF_DIR     20  // RS-485 DE/RE direction control (shared, TX-only)

// total output channels: 8 native + 4 differential = 12
#define TOTAL_PORTS (NUM_PORTS + NUM_DIFF_PORTS)

// ---------------------------------------------------------------------------
// DMX512 output (1x RJ45 via RS-485 driver)
// GPIO21 is UART1_RX (not TX) on RP2350, so hardware UART cannot be used.
// PIO-based UART TX on pio2/SM0 instead (standard RP2040/RP2350 pattern).
// ---------------------------------------------------------------------------
#define PIN_DMX_TX       21  // PIO UART TX -> RS-485 driver
#define PIN_DMX_DIR      22  // RS-485 DE/RE direction control

// ---------------------------------------------------------------------------
// SPI0 slave pins (CM5 SPI0 CE0 -> RP2350 SPI0)
// ---------------------------------------------------------------------------
#define SPI_PORT       spi0
#define SPI_RESET_BITS RESETS_RESET_SPI0_BITS
#define PIN_SPI_RX     0   // SPI0_RX: slave input  <- CM5 MOSI
#define PIN_SPI_CS     1   // SPI0_CSn
#define PIN_SPI_SCK    2   // SPI0_SCK
#define PIN_SPI_TX     3   // SPI0_TX: slave output -> CM5 MISO

// READY signal: HIGH = ready for data, LOW = processing
#define PIN_READY 4

// ---------------------------------------------------------------------------
// Status LED (RP2350 firmware heartbeat / activity indicator)
// ---------------------------------------------------------------------------
#define PIN_LED 5

// ---------------------------------------------------------------------------
// Debug UART (optional, active during development)
// ---------------------------------------------------------------------------
// #define PIN_DEBUG_TX  6
// #define PIN_DEBUG_RX  7

// max frame size: all 12 channels' pixel data + max DMX payload.
// CM5 sends all channels (8 native + 4 differential) to this RP2350.
// DMX overhead: 512 data bytes + 2-byte length field = 514 bytes.
#define MAX_FRAME_SIZE (TOTAL_PORTS * MAX_PIXELS * 3 + 514)

#endif
