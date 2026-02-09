#ifndef SIGNAL_BOARD_CONFIG_H
#define SIGNAL_BOARD_CONFIG_H

// board identity
#define BOARD_NAME "pico2w-8ch"

// system clock: 180 MHz for SPI slave oversampling headroom
#define SYS_CLOCK_KHZ 180000

// WS281x PIO output (GPIO0-7 through SN74AHCT245N level shifter)
#define NEO_PIN_BASE 0
#define NUM_PORTS    8
#define MAX_PIXELS   800

// SPI0 slave pins (GPIO16-19, leaves GPIO8-15 free for 16-ch expansion)
#define SPI_PORT       spi0
#define SPI_RESET_BITS RESETS_RESET_SPI0_BITS
#define PIN_SPI_RX     16  // SPI0_RX: slave input  <- Pi MOSI
#define PIN_SPI_CS     17
#define PIN_SPI_SCK    18
#define PIN_SPI_TX     19  // SPI0_TX: slave output -> Pi MISO

// READY signal: HIGH = ready for data, LOW = processing
#define PIN_READY 20

// Pico 2 W has no regular GPIO onboard LED (it's on CYW43).
// define PIN_LED here if an external LED is wired to a spare GPIO.
// PIN_NEOPIXEL intentionally NOT defined (no onboard NeoPixel).

// max frame size (derived from board capacity)
#define MAX_FRAME_SIZE (NUM_PORTS * MAX_PIXELS * 3)

#endif
