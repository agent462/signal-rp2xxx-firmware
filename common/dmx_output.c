#include "dmx_output.h"
#include "dmx_output.pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

// DMX512 timing constants
#define DMX_BAUD       250000
#define DMX_BREAK_US   100   // BREAK: ≥88µs LOW (we use 100µs)
#define DMX_MAB_US     16    // Mark After Break: ≥8µs HIGH (we use 16µs)
#define DMX_START_CODE 0x00  // DMX start code (slot 0)

void dmx_output_init(dmx_ctx_t *ctx, PIO pio, uint sm,
                     uint pin_tx, uint pin_dir) {
    ctx->pio = pio;
    ctx->sm = sm;
    ctx->pin_tx = pin_tx;
    ctx->pin_dir = pin_dir;

    // load PIO UART TX program
    ctx->offset = pio_add_program(pio, &dmx_uart_tx_program);

    pio_sm_config c = dmx_uart_tx_program_get_default_config(ctx->offset);

    // TX pin: both OUT and SET use the same pin
    sm_config_set_out_pins(&c, pin_tx, 1);
    sm_config_set_set_pins(&c, pin_tx, 1);

    // shift right (LSB first), manual pull via explicit 'pull block' in PIO program
    sm_config_set_out_shift(&c, true, false, 8);

    // clock divider: 8 PIO cycles = 1 bit time at 250kbaud
    float div = (float)clock_get_hz(clk_sys) / ((float)DMX_BAUD * 8.0f);
    sm_config_set_clkdiv(&c, div);

    // configure TX GPIO for PIO
    pio_gpio_init(pio, pin_tx);
    pio_sm_set_consecutive_pindirs(pio, sm, pin_tx, 1, true);

    // initialize SM (idle HIGH = UART mark state)
    pio_sm_init(pio, sm, ctx->offset, &c);

    // RS-485 direction pin: LOW = receive (idle)
    gpio_init(pin_dir);
    gpio_set_dir(pin_dir, GPIO_OUT);
    gpio_put(pin_dir, 0);
}

void dmx_output_send(dmx_ctx_t *ctx, const uint8_t *data, uint len) {
    if (len == 0 || len > 512) return;

    // assert RS-485 direction: transmit
    gpio_put(ctx->pin_dir, 1);

    // --- BREAK: take pin from PIO, drive LOW via SIO ---
    pio_sm_set_enabled(ctx->pio, ctx->sm, false);

    // switch pin from PIO to SIO control
    gpio_init(ctx->pin_tx);
    gpio_set_dir(ctx->pin_tx, GPIO_OUT);
    gpio_put(ctx->pin_tx, 0);
    sleep_us(DMX_BREAK_US);

    // --- MAB: drive HIGH via SIO ---
    gpio_put(ctx->pin_tx, 1);
    sleep_us(DMX_MAB_US);

    // --- return pin to PIO, restart SM ---
    pio_gpio_init(ctx->pio, ctx->pin_tx);
    pio_sm_restart(ctx->pio, ctx->sm);
    pio_sm_exec(ctx->pio, ctx->sm, pio_encode_jmp(ctx->offset));
    pio_sm_set_enabled(ctx->pio, ctx->sm, true);

    // --- send start code + data bytes ---
    pio_sm_put_blocking(ctx->pio, ctx->sm, DMX_START_CODE);
    for (uint i = 0; i < len; i++) {
        pio_sm_put_blocking(ctx->pio, ctx->sm, data[i]);
    }

    // wait for last byte to finish transmitting.
    // TX FIFO empty doesn't mean the last byte has shifted out.
    // wait for FIFO empty + one byte drain time (44µs at 250kbaud).
    while (!pio_sm_is_tx_fifo_empty(ctx->pio, ctx->sm)) {
        tight_loop_contents();
    }
    sleep_us(50);  // 44µs byte time + polling/startup margin

    // de-assert RS-485 direction: back to receive/idle
    gpio_put(ctx->pin_dir, 0);
}
