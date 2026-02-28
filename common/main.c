#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware/dma.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"
#include "config.h"
#include "spi_slave.h"
#include "frame_protocol.h"
#include "ws281x_parallel.h"
#include "bitplane.h"
#include "status_led.h"
#ifdef PIN_DMX_TX
#include "dmx_output.h"
#endif

// multicore FIFO tokens
#define CORE1_READY   0xC0DE0001
#define CORE1_DONE    0xC0DE0002

// software timeout: log warning after 2s with no valid frames
#define TIMEOUT_US 2000000

// hardware watchdog: reboot if main loop hangs for 5s
#define WATCHDOG_MS 5000

// frame dispatch struct: passed from core0 to core1 via FIFO pointer.
// single pointer avoids needing multiple FIFO pushes and carries
// both pixel and optional DMX parameters.
typedef struct {
    const uint8_t *pixel_data;
    uint32_t num_ports;
    uint32_t pixels_per_port;
#ifdef PIN_DMX_TX
    bool dmx_present;
    const uint8_t *dmx_data;
    uint32_t dmx_len;
#endif
} frame_dispatch_t;

// core1: bitplane transform + PIO WS281x output + optional DMX.
// receives frame dispatch struct from core0 via multicore FIFO, transforms
// pixel data into bitplane format, and DMA-feeds the PIO state machine(s).
static void core1_main(void) {
    // initialize native WS281x PIO output (pio0/SM0)
    static ws281x_ctx_t neo_ctx;
    ws281x_parallel_init(&neo_ctx, pio0, 0, NEO_PIN_BASE, NUM_PORTS, 800000.0f);

#ifdef NUM_DIFF_PORTS
    // initialize differential WS281x PIO output (pio1/SM0)
    static ws281x_ctx_t diff_ctx;
    ws281x_parallel_init(&diff_ctx, pio1, 0, DIFF_PIN_BASE, NUM_DIFF_PORTS, 800000.0f);

    // RS-485 direction pin: LOW = receive (idle), HIGH = transmit
    gpio_init(PIN_DIFF_DIR);
    gpio_set_dir(PIN_DIFF_DIR, GPIO_OUT);
    gpio_put(PIN_DIFF_DIR, 0);

    // enable both SMs (sequential start, <1µs skew -- negligible at 1.25µs/bit)
    pio_sm_set_enabled(pio0, 0, true);
    pio_sm_set_enabled(pio1, 0, true);
#else
    pio_sm_set_enabled(pio0, 0, true);
#endif

#ifdef PIN_DMX_TX
    // initialize DMX512 PIO UART output (pio2/SM0)
    static dmx_ctx_t dmx_ctx;
    dmx_output_init(&dmx_ctx, pio2, 0, PIN_DMX_TX, PIN_DMX_DIR);
#endif

    static uint32_t bitplane_neo[BITPLANE_WORDS(MAX_PIXELS)];
#ifdef NUM_DIFF_PORTS
    static uint32_t bitplane_diff[BITPLANE_WORDS(MAX_PIXELS)];
#endif

    // signal core0: PIO initialized, ready for frames
    multicore_fifo_push_blocking(CORE1_READY);

    while (true) {
        // block until core0 sends a frame dispatch struct pointer
        uint32_t dispatch_ptr = multicore_fifo_pop_blocking();
        __mem_fence_acquire();  // ensure struct reads happen after pointer received
        const frame_dispatch_t *dispatch = (const frame_dispatch_t *)dispatch_ptr;

        uint32_t num_ports = dispatch->num_ports;
        uint32_t pixels_per_port = dispatch->pixels_per_port;
        const uint8_t *pixel_data = dispatch->pixel_data;

#ifdef NUM_DIFF_PORTS
        // native ports (0..NUM_PORTS-1): use standard transform for up to 8 ports
        if (num_ports <= NUM_PORTS) {
            bitplane_transform(pixel_data, bitplane_neo, num_ports, pixels_per_port);
        } else {
            // frame has more ports than native -- split: native gets ports 0..7
            bitplane_transform_subset(pixel_data, bitplane_neo,
                                      0, NUM_PORTS, pixels_per_port);
            // differential gets ports 8..11
            uint32_t diff_ports = num_ports - NUM_PORTS;
            if (diff_ports > NUM_DIFF_PORTS) diff_ports = NUM_DIFF_PORTS;
            bitplane_transform_subset(pixel_data, bitplane_diff,
                                      NUM_PORTS, diff_ports, pixels_per_port);
        }
#else
        bitplane_transform(pixel_data, bitplane_neo, num_ports, pixels_per_port);
#endif

#ifdef PIN_DMX_TX
        // copy DMX data before signaling CORE1_DONE -- dmx_data points into
        // the SPI buffer, which core0 will re-arm after receiving CORE1_DONE.
        static uint8_t dmx_buf[512];
        uint32_t dmx_buf_len = 0;
        bool dmx_pending = false;
        if (dispatch->dmx_present && dispatch->dmx_len <= sizeof(dmx_buf)) {
            memcpy(dmx_buf, dispatch->dmx_data, dispatch->dmx_len);
            dmx_buf_len = dispatch->dmx_len;
            dmx_pending = true;
        }
#endif

        // signal core0: done reading from SPI buffer, safe to re-arm DMA
        multicore_fifo_push_blocking(CORE1_DONE);

        uint32_t num_words = BITPLANE_WORDS(pixels_per_port);

#ifdef NUM_DIFF_PORTS
        if (num_ports > NUM_PORTS) {
            // assert RS-485 direction for differential output
            gpio_put(PIN_DIFF_DIR, 1);

            // start both DMA transfers concurrently
            dma_channel_set_read_addr(neo_ctx.dma_chan, bitplane_neo, false);
            dma_channel_set_trans_count(neo_ctx.dma_chan, num_words, true);
            dma_channel_set_read_addr(diff_ctx.dma_chan, bitplane_diff, false);
            dma_channel_set_trans_count(diff_ctx.dma_chan, num_words, true);

            // wait for both DMA transfers to complete
            dma_channel_wait_for_finish_blocking(neo_ctx.dma_chan);
            dma_channel_wait_for_finish_blocking(diff_ctx.dma_chan);

            // wait for differential PIO to finish shifting out FIFO contents.
            // DMA done = all words written to FIFO, not shifted to pins.
            // FIFO depth is 4 words * 1.25µs/word = up to 5µs remaining.
            while (!pio_sm_is_tx_fifo_empty(pio1, 0)) {
                tight_loop_contents();
            }
            sleep_us(40);  // final word shift-out (24 bits * 1.25µs ≈ 30µs + margin)

            // de-assert RS-485 direction
            gpio_put(PIN_DIFF_DIR, 0);
        } else {
            // native only -- no differential output
            ws281x_parallel_send(&neo_ctx, bitplane_neo, num_words);
        }
#else
        ws281x_parallel_send(&neo_ctx, bitplane_neo, num_words);
#endif

#ifdef PIN_DMX_TX
        if (dmx_pending) {
            dmx_output_send(&dmx_ctx, dmx_buf, dmx_buf_len);
        }
#endif

        // WS281x latch: data is latched on >280us LOW (WS2812B spec).
        // the next frame's SPI reception provides natural delay, but
        // add a minimum guard for back-to-back frames.
        sleep_us(300);
    }
}

int main(void) {
    set_sys_clock_khz(SYS_CLOCK_KHZ, true);
    stdio_init_all();

#ifdef PIN_LED
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
#endif

    status_led_init();
    status_led_set(STATUS_IDLE);
    // force initial display of idle state
    status_led_update(0);

    sleep_ms(2000);
    printf("=== signal %s (dual-core SPI+WS281x) ===\n", BOARD_NAME);
    printf("sys clock: %lu Hz\n", (unsigned long)clock_get_hz(clk_sys));

    spi_slave_init();
    printf("SPI slave initialized\n");

    // launch core1 and wait for PIO init to complete
    multicore_launch_core1(core1_main);
    uint32_t ready = multicore_fifo_pop_blocking();
    if (ready != CORE1_READY) {
        printf("ERROR: unexpected core1 ready token: 0x%08lx\n",
               (unsigned long)ready);
    }
    printf("core1 PIO initialized, waiting for frames...\n");

    // enable hardware watchdog (5s safety net for firmware hangs)
    watchdog_enable(WATCHDOG_MS, true);

    // frame dispatch struct -- core1 reads pixel_data and copies dmx_data
    // before signaling CORE1_DONE, so it's always valid.
    static frame_dispatch_t dispatch;

    uint32_t frame_count = 0;
    uint32_t error_count = 0;
    uint32_t last_frame_num = 0;
    uint32_t drop_count = 0;
    uint64_t last_valid_frame_us = time_us_64();
    bool timed_out = false;

    while (true) {
        watchdog_update();

        if (!spi_slave_frame_ready()) {
            // check software timeout in idle path
            if (!timed_out && (time_us_64() - last_valid_frame_us) > TIMEOUT_US) {
                timed_out = true;
                status_led_set(STATUS_TIMEOUT);
                status_led_update(frame_count);
                printf("timeout: no frames for 2s\n");
            }

            tight_loop_contents();
            continue;
        }

        const uint8_t *data;
        uint32_t len;
        if (!spi_slave_get_frame(&data, &len)) {
            spi_slave_frame_consumed();
            continue;
        }

        // parse and validate frame (sync, length, ports, CRC)
        frame_info_t info;
        frame_result_t result = frame_parse(data, len, &info);

        if (result != FRAME_OK) {
            error_count++;
            status_led_set(STATUS_CRC_ERROR);
            status_led_update(error_count);
            // log first error and then every 100th to avoid serial flooding
            if (error_count == 1 || error_count % 100 == 0) {
                printf("frame error %d, total errors: %lu\n",
                       result, (unsigned long)error_count);
            }
            spi_slave_frame_consumed();
            continue;
        }

        frame_count++;
#ifdef PIN_LED
        gpio_put(PIN_LED, frame_count & 1);
#endif

        // track last valid frame time for timeout detection
        last_valid_frame_us = time_us_64();

        if (timed_out) {
            timed_out = false;
            printf("frames resumed\n");
        }

        // set status based on error history
        status_led_set(STATUS_RECEIVING);
        status_led_update(frame_count);

        // detect frame number gaps (dropped frames).
        // if frame_num <= last_frame_num, the sender restarted -- reset tracking.
        if (info.frame_num <= last_frame_num) {
            last_frame_num = 0;
        }
        if (last_frame_num > 0 && info.frame_num > last_frame_num + 1) {
            uint32_t gap = info.frame_num - last_frame_num - 1;
            drop_count += gap;
        }
        last_frame_num = info.frame_num;

        // populate dispatch struct and hand to core1 via single FIFO push
        dispatch.pixel_data = info.pixel_data;
        dispatch.num_ports = info.num_ports;
        dispatch.pixels_per_port = info.pixels_per_port;
#ifdef PIN_DMX_TX
        dispatch.dmx_present = info.dmx_present;
        dispatch.dmx_data = info.dmx_data;
        dispatch.dmx_len = info.dmx_len;
#endif
        __mem_fence_release();  // ensure all struct writes complete before pointer is visible
        multicore_fifo_push_blocking((uint32_t)&dispatch);

        // wait for core1 to finish reading from the SPI buffer.
        // core1 pushes CORE1_DONE after bitplane_transform() completes.
        // PIO DMA output continues from core1's own bitplane buffer.
        multicore_fifo_pop_blocking();

        // re-arm SPI for next frame (swaps buffer, sets READY HIGH)
        spi_slave_frame_consumed();

        // periodic stats (every 1000 frames, ~25s at 40fps)
        if (frame_count % 1000 == 0) {
            printf("frames: %lu, errors: %lu, drops: %lu, "
                   "ports: %lu, px/port: %lu\n",
                   (unsigned long)frame_count,
                   (unsigned long)error_count,
                   (unsigned long)drop_count,
                   (unsigned long)info.num_ports,
                   (unsigned long)info.pixels_per_port);
        }
    }

    return 0;
}
