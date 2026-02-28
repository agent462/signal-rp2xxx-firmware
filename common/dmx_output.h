#ifndef DMX_OUTPUT_H
#define DMX_OUTPUT_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/pio.h"

typedef struct {
    PIO pio;
    uint sm;
    uint pin_tx;
    uint pin_dir;
    uint offset;  // PIO program offset for restart after BREAK
} dmx_ctx_t;

// initialize PIO-based DMX512 UART TX.
// ctx: context struct to populate
// pio: PIO instance (pio2 on Signal 8)
// sm: state machine number
// pin_tx: GPIO for UART TX output
// pin_dir: GPIO for RS-485 direction control (DE/RE)
void dmx_output_init(dmx_ctx_t *ctx, PIO pio, uint sm,
                     uint pin_tx, uint pin_dir);

// send a DMX512 frame: BREAK + MAB + start code (0x00) + data.
// blocks until the entire frame has been transmitted.
// data: DMX channel data (1-512 bytes)
// len: number of data bytes (1-512)
void dmx_output_send(dmx_ctx_t *ctx, const uint8_t *data, uint len);

#endif
