#ifndef SIGNAL_BOARD_CONFIG_H
#define SIGNAL_BOARD_CONFIG_H

// board identity
#define BOARD_NAME "pico2w-16ch"

// system clock: 288 MHz (48 * 6, clean USB divisor).
// PL022 slave requires clk_peri >= 12 * f_SCK; at 288 MHz,
// max SPI slave clock = 288 / 12 = 24 MHz.
#define SYS_CLOCK_KHZ 288000

// WS281x PIO output (GPIO0-15 through SN74AHCT245N level shifters)
#define NEO_PIN_BASE 0
#define NUM_PORTS    16
#define MAX_PIXELS   800

// SPI0 slave pins (GPIO16-19, same wiring as 8-ch)
#define SPI_PORT       spi0
#define SPI_RESET_BITS RESETS_RESET_SPI0_BITS
#define PIN_SPI_RX     16  // SPI0_RX: slave input  <- Pi MOSI
#define PIN_SPI_CS     17
#define PIN_SPI_SCK    18
#define PIN_SPI_TX     19  // SPI0_TX: slave output -> Pi MISO

// READY signal: HIGH = ready for data, LOW = processing
#define PIN_READY 20

// max frame size (derived from board capacity)
#define MAX_FRAME_SIZE (NUM_PORTS * MAX_PIXELS * 3)

#endif
