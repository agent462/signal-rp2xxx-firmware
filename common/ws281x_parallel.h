#ifndef WS281X_PARALLEL_H
#define WS281X_PARALLEL_H

#include <stdint.h>
#include "hardware/pio.h"

// context for one PIO+DMA WS281x output instance.
// supports multiple independent instances (e.g. native + differential).
typedef struct {
    PIO pio;
    uint sm;
    int dma_chan;
} ws281x_ctx_t;

// initialize PIO state machine and DMA channel for parallel WS281x output.
// ctx: context struct to populate (caller owns storage)
// pio: PIO instance (pio0, pio1, pio2)
// sm: state machine number (0-3)
// pin_base: first GPIO pin
// pin_count: number of output pins (up to 8)
// freq: WS281x bit frequency in Hz (800000 for WS2812/WS2811)
void ws281x_parallel_init(ws281x_ctx_t *ctx, PIO pio, uint sm,
                          uint pin_base, uint pin_count, float freq);

// send bitplane data to LEDs via DMA. blocks until transfer completes.
// ctx: initialized context
// bitplane: array of 32-bit words in bitplane format (1 bit per strip per word)
// num_words: number of words to send (pixels_per_port * 24)
void ws281x_parallel_send(ws281x_ctx_t *ctx, const uint32_t *bitplane, uint num_words);

#endif
