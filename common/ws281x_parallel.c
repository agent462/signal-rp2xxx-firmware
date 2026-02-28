#include "ws281x_parallel.h"
#include "ws281x_parallel.pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"

void ws281x_parallel_init(ws281x_ctx_t *ctx, PIO pio, uint sm,
                          uint pin_base, uint pin_count, float freq) {
    ctx->pio = pio;
    ctx->sm = sm;

    // load PIO program into instruction memory
    uint offset = pio_add_program(pio, &ws281x_parallel_program);

    pio_sm_config c = ws281x_parallel_program_get_default_config(offset);

    // map OUT pins to our LED GPIOs (used by mov pins)
    sm_config_set_out_pins(&c, pin_base, pin_count);

    // autopull: shift left, pull threshold 32 bits
    sm_config_set_out_shift(&c, false, true, 32);

    // clock divider: 10 PIO cycles per WS281x bit
    float div = (float)clock_get_hz(clk_sys) / (freq * 10.0f);
    sm_config_set_clkdiv(&c, div);

    // configure GPIO pins for PIO output
    for (uint i = 0; i < pin_count; i++) {
        pio_gpio_init(pio, pin_base + i);
    }
    pio_sm_set_consecutive_pindirs(pio, sm, pin_base, pin_count, true);

    // initialize state machine (don't enable yet -- caller controls start)
    pio_sm_init(pio, sm, offset, &c);

    // claim a DMA channel for feeding the PIO TX FIFO
    ctx->dma_chan = dma_claim_unused_channel(true);
    dma_channel_config dc = dma_channel_get_default_config(ctx->dma_chan);
    channel_config_set_transfer_data_size(&dc, DMA_SIZE_32);
    channel_config_set_read_increment(&dc, true);
    channel_config_set_write_increment(&dc, false);
    channel_config_set_dreq(&dc, pio_get_dreq(pio, sm, true));

    dma_channel_configure(ctx->dma_chan, &dc,
        &pio->txf[sm],  // write: PIO TX FIFO
        NULL,            // read: set per transfer
        0,               // count: set per transfer
        false            // don't start yet
    );
}

void ws281x_parallel_send(ws281x_ctx_t *ctx, const uint32_t *bitplane, uint num_words) {
    // wait for any previous transfer to complete
    dma_channel_wait_for_finish_blocking(ctx->dma_chan);

    // start new transfer
    dma_channel_set_read_addr(ctx->dma_chan, bitplane, false);
    dma_channel_set_trans_count(ctx->dma_chan, num_words, true);

    // block until this transfer completes
    dma_channel_wait_for_finish_blocking(ctx->dma_chan);
}
