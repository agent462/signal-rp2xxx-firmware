#ifndef SIGNAL_BOARD_CONFIG_H
#define SIGNAL_BOARD_CONFIG_H

// board identity
#define BOARD_NAME "pico2w-24ch"

// system clock: 288 MHz (48 * 6, clean USB divisor).
// PL022 slave requires clk_peri >= 12 * f_SCK; at 288 MHz,
// max SPI slave clock = 288 / 12 = 24 MHz.
#define SYS_CLOCK_KHZ 288000

// WS281x PIO output (GPIO0-23 through SN74AHCT245N level shifters)
#define NEO_PIN_BASE 0
#define NUM_PORTS    24
#define MAX_PIXELS   800

// SPI1 slave pins (GPIO24-27, moved from SPI0 since GPIO16-19 are pixel outputs)
#define SPI_PORT       spi1
#define SPI_RESET_BITS RESETS_RESET_SPI1_BITS
#define PIN_SPI_RX     24  // SPI1_RX: slave input  <- Pi MOSI
#define PIN_SPI_CS     25
#define PIN_SPI_SCK    26
#define PIN_SPI_TX     27  // SPI1_TX: slave output -> Pi MISO

// READY signal: HIGH = ready for data, LOW = processing
#define PIN_READY 28

// max frame size (derived from board capacity)
#define MAX_FRAME_SIZE (NUM_PORTS * MAX_PIXELS * 3)

#endif
