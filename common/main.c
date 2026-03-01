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
// core0 pre-computes bitplane data; core1 just does DMA output.
typedef struct {
    const uint32_t *bitplane_neo;
    uint32_t num_words;
#ifdef NUM_DIFF_PORTS
    const uint32_t *bitplane_diff;
    bool has_diff;
#endif
#ifdef PIN_DMX_TX
    bool dmx_present;
    const uint8_t *dmx_data;
    uint32_t dmx_len;
#endif
} frame_dispatch_t;

// double-buffered bitplane arrays: core0 writes one while core1 DMA-reads the other
static uint32_t bitplane_neo[2][BITPLANE_WORDS(MAX_PIXELS)];
static int bp_write = 0;

#ifdef NUM_DIFF_PORTS
static uint32_t bitplane_diff[2][BITPLANE_WORDS(MAX_PIXELS)];
#endif

// core1: PIO WS281x DMA output + optional DMX.
// receives pre-computed bitplane data from core0 via multicore FIFO and
// DMA-feeds the PIO state machine(s).
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

    // signal core0: PIO initialized, ready for frames
    multicore_fifo_push_blocking(CORE1_READY);

    while (true) {
        // block until core0 sends a frame dispatch struct pointer
        uint32_t dispatch_ptr = multicore_fifo_pop_blocking();
        __mem_fence_acquire();
        const frame_dispatch_t *dispatch = (const frame_dispatch_t *)dispatch_ptr;

        uint32_t num_words = dispatch->num_words;

#ifdef NUM_DIFF_PORTS
        if (dispatch->has_diff) {
            gpio_put(PIN_DIFF_DIR, 1);

            // start both DMA transfers concurrently
            dma_channel_set_read_addr(neo_ctx.dma_chan, dispatch->bitplane_neo, false);
            dma_channel_set_trans_count(neo_ctx.dma_chan, num_words, true);
            dma_channel_set_read_addr(diff_ctx.dma_chan, dispatch->bitplane_diff, false);
            dma_channel_set_trans_count(diff_ctx.dma_chan, num_words, true);

            dma_channel_wait_for_finish_blocking(neo_ctx.dma_chan);
            dma_channel_wait_for_finish_blocking(diff_ctx.dma_chan);

            // wait for differential PIO to finish shifting out FIFO contents
            while (!pio_sm_is_tx_fifo_empty(pio1, 0)) {
                tight_loop_contents();
            }
            sleep_us(40);

            gpio_put(PIN_DIFF_DIR, 0);
        } else {
            ws281x_parallel_send(&neo_ctx, dispatch->bitplane_neo, num_words);
        }
#else
        ws281x_parallel_send(&neo_ctx, dispatch->bitplane_neo, num_words);
#endif

#ifdef PIN_DMX_TX
        if (dispatch->dmx_present) {
            dmx_output_send(&dmx_ctx, dispatch->dmx_data, dispatch->dmx_len);
        }
#endif

        // WS281x latch: data is latched on >280µs LOW (WS2812B spec).
        // bitplane transform moved to core0, so explicit latch delay needed.
        sleep_us(300);

        // signal core0: DMA complete, bitplane buffer safe to reuse
        multicore_fifo_push_blocking(CORE1_DONE);
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
    printf("peri clock: %lu Hz\n", (unsigned long)clock_get_hz(clk_peri));

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

    static frame_dispatch_t dispatch;
#ifdef PIN_DMX_TX
    static uint8_t dmx_buf[2][512];
#endif

    uint32_t frame_count = 0;
    uint32_t error_count = 0;
    uint32_t last_frame_num = 0;
    uint32_t drop_count = 0;
    uint64_t total_bytes = 0;
    uint32_t err_counts[6] = {0};  // per-type: TOO_SHORT, BAD_SYNC, BAD_LENGTH, BAD_PORTS, BAD_CRC, ALIGNMENT
    uint64_t stats_interval_start_us = time_us_64();
    uint32_t interval_frames = 0;
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
            if (result >= FRAME_ERR_TOO_SHORT && result <= FRAME_ERR_ALIGNMENT)
                err_counts[result - 1]++;
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
        total_bytes += len;
        interval_frames++;
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

        // bitplane transform into write buffer (on core0, overlaps with core1 DMA)
#ifdef NUM_DIFF_PORTS
        if (info.num_ports <= NUM_PORTS) {
            bitplane_transform(info.pixel_data, bitplane_neo[bp_write],
                               info.num_ports, info.pixels_per_port);
            dispatch.has_diff = false;
        } else {
            bitplane_transform_subset(info.pixel_data, bitplane_neo[bp_write],
                                      0, NUM_PORTS, info.pixels_per_port);
            uint32_t diff_ports = info.num_ports - NUM_PORTS;
            if (diff_ports > NUM_DIFF_PORTS) diff_ports = NUM_DIFF_PORTS;
            bitplane_transform_subset(info.pixel_data, bitplane_diff[bp_write],
                                      NUM_PORTS, diff_ports, info.pixels_per_port);
            dispatch.has_diff = true;
        }
#else
        bitplane_transform(info.pixel_data, bitplane_neo[bp_write],
                           info.num_ports, info.pixels_per_port);
#endif

        // copy DMX data from SPI buffer into staging buffer
#ifdef PIN_DMX_TX
        dispatch.dmx_present = info.dmx_present;
        if (info.dmx_present && info.dmx_len <= 512) {
            memcpy(dmx_buf[bp_write], info.dmx_data, info.dmx_len);
            dispatch.dmx_data = dmx_buf[bp_write];
            dispatch.dmx_len = info.dmx_len;
        }
#endif

        // populate dispatch struct with pre-computed bitplane pointers
        dispatch.bitplane_neo = bitplane_neo[bp_write];
        dispatch.num_words = BITPLANE_WORDS(info.pixels_per_port);
#ifdef NUM_DIFF_PORTS
        dispatch.bitplane_diff = bitplane_diff[bp_write];
#endif

        // SPI buffer fully consumed — re-arm DMA, READY goes HIGH.
        // this lets the Pi start sending the next frame while we wait for core1.
        spi_slave_frame_consumed();

        // wait for core1 to finish previous DMA (bitplane read buffer now safe)
        if (frame_count > 1) {
            multicore_fifo_pop_blocking();  // CORE1_DONE
        }

        // dispatch to core1
        __mem_fence_release();
        multicore_fifo_push_blocking((uint32_t)&dispatch);
        bp_write ^= 1;

        // periodic stats (every 1000 frames, ~25s at 40fps)
        if (frame_count % 1000 == 0) {
            uint64_t now_us = time_us_64();
            uint64_t elapsed_us = now_us - stats_interval_start_us;
            uint32_t fps_x10 = (elapsed_us > 0)
                ? (uint32_t)((uint64_t)interval_frames * 10000000 / elapsed_us)
                : 0;
            uint32_t uptime_s = (uint32_t)(now_us / 1000000);

            printf("[%lus] frames:%lu err:%lu drop:%lu fps:%lu.%lu bytes:%llu "
                   "dma_rem:%lu fifo_extra:%lu "
                   "err[sync:%lu crc:%lu len:%lu port:%lu align:%lu short:%lu]\n",
                   (unsigned long)uptime_s,
                   (unsigned long)frame_count,
                   (unsigned long)error_count,
                   (unsigned long)drop_count,
                   (unsigned long)(fps_x10 / 10),
                   (unsigned long)(fps_x10 % 10),
                   (unsigned long long)total_bytes,
                   (unsigned long)spi_slave_debug_remaining(),
                   (unsigned long)spi_slave_debug_fifo_extra(),
                   (unsigned long)err_counts[FRAME_ERR_BAD_SYNC - 1],
                   (unsigned long)err_counts[FRAME_ERR_BAD_CRC - 1],
                   (unsigned long)err_counts[FRAME_ERR_BAD_LENGTH - 1],
                   (unsigned long)err_counts[FRAME_ERR_BAD_PORTS - 1],
                   (unsigned long)err_counts[FRAME_ERR_ALIGNMENT - 1],
                   (unsigned long)err_counts[FRAME_ERR_TOO_SHORT - 1]);

            interval_frames = 0;
            stats_interval_start_us = now_us;
        }
    }

    return 0;
}
