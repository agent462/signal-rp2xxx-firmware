#ifndef SIGNAL_BOARD_CONFIG_H
#define SIGNAL_BOARD_CONFIG_H

// board identity
#define BOARD_NAME "scorpio"

// system clock: 180 MHz for SPI slave oversampling headroom
#define SYS_CLOCK_KHZ 180000

// WS281x LED outputs (directly from RP2040 GPIO, active HIGH)
#define NEO_PIN_BASE 16
#define NUM_PORTS    8
#define MAX_PIXELS   800

// SPI1 slave pins (directly wired from Pi SPI0 master)
#define SPI_PORT       spi1
#define SPI_RESET_BITS RESETS_RESET_SPI1_BITS
#define PIN_SPI_SCK    14
#define PIN_SPI_TX     15  // SPI1_TX: slave output -> Pi MISO (GPIO15 = MO on SCORPIO)
#define PIN_SPI_RX     8   // SPI1_RX: slave input  <- Pi MOSI (GPIO08 = MI on SCORPIO)
#define PIN_SPI_CS     9

// READY signal: HIGH = ready for data, LOW = processing
#define PIN_READY 10

// onboard LEDs
#define PIN_LED       13  // built-in red LED
#define PIN_NEOPIXEL  4   // onboard RGB NeoPixel (status)

// max frame size (derived from board capacity)
#define MAX_FRAME_SIZE (NUM_PORTS * MAX_PIXELS * 3)

#endif
