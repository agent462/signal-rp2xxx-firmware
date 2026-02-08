#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware/watchdog.h"
#include "config.h"
#include "spi_slave.h"
#include "frame_protocol.h"
#include "ws281x_parallel.h"
#include "bitplane.h"
#include "status_led.h"

// multicore FIFO tokens
#define CORE1_READY   0xC0DE0001
#define CORE1_DONE    0xC0DE0002

// software timeout: blank LEDs after 2s with no valid frames
#define TIMEOUT_US 2000000

// hardware watchdog: reboot if main loop hangs for 5s
#define WATCHDOG_MS 5000

// static blank pixel buffer for timeout blanking
static uint8_t blank_pixels[MAX_FRAME_SIZE];

// core1: bitplane transform + PIO WS281x output.
// receives frame pointers from core0 via multicore FIFO, transforms pixel
// data into bitplane format, and DMA-feeds the PIO state machine.
static void core1_main(void) {
    // initialize WS281x PIO output on this core (owns the PIO DMA channel)
    ws281x_parallel_init(pio0, 0, NEO_PIN_BASE, NUM_PORTS, 800000.0f);

    static uint32_t bitplane[BITPLANE_WORDS(MAX_PIXELS)];

    // signal core0: PIO initialized, ready for frames
    multicore_fifo_push_blocking(CORE1_READY);

    while (true) {
        // block until core0 sends a validated frame
        uint32_t data_ptr = multicore_fifo_pop_blocking();
        uint32_t params = multicore_fifo_pop_blocking();

        uint32_t num_ports = params >> 16;
        uint32_t pixels_per_port = params & 0xFFFF;
        const uint8_t *pixel_data = (const uint8_t *)data_ptr;

        // transform pixel data into bitplane format.
        // reads from the SPI DMA buffer -- must complete before core0 re-arms.
        bitplane_transform(pixel_data, bitplane, num_ports, pixels_per_port);

        // signal core0: done reading from SPI buffer, safe to re-arm DMA
        multicore_fifo_push_blocking(CORE1_DONE);

        // output bitplanes to LED strips via PIO DMA (blocks until complete)
        uint32_t num_words = BITPLANE_WORDS(pixels_per_port);
        ws281x_parallel_send(pio0, 0, bitplane, num_words);

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
    printf("=== signal scorpio phase 6 (dual-core SPI+WS281x) ===\n");
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

    // zero the blank buffer once (BSS is zero-initialized, but be explicit)
    memset(blank_pixels, 0, sizeof(blank_pixels));

    // enable hardware watchdog (5s safety net for firmware hangs)
    watchdog_enable(WATCHDOG_MS, true);

    uint32_t frame_count = 0;
    uint32_t error_count = 0;
    uint32_t last_frame_num = 0;
    uint32_t drop_count = 0;
    uint64_t last_valid_frame_us = time_us_64();
    bool timed_out = false;
    uint32_t last_num_ports = NUM_PORTS;
    uint32_t last_pixels_per_port = MAX_PIXELS;

    while (true) {
        watchdog_update();

        if (!spi_slave_frame_ready()) {
            // check software timeout in idle path
            if (!timed_out && (time_us_64() - last_valid_frame_us) > TIMEOUT_US) {
                timed_out = true;
                status_led_set(STATUS_TIMEOUT);
                status_led_update(frame_count);

                // send blank frame to core1 via FIFO.
                // use timeout push to handle potential core1 hang.
                if (multicore_fifo_push_timeout_us(
                        (uint32_t)blank_pixels, 100000)) {
                    multicore_fifo_push_blocking(
                        (last_num_ports << 16) | last_pixels_per_port);
                    // wait for core1 to finish reading blank buffer
                    multicore_fifo_pop_blocking();
                }

                printf("timeout: no frames for 2s, blanked LEDs\n");
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

        // track last valid frame time and dimensions for timeout blanking
        last_valid_frame_us = time_us_64();
        last_num_ports = info.num_ports;
        last_pixels_per_port = info.pixels_per_port;

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

        // hand frame to core1 for LED output:
        // push pixel data pointer, then (ports << 16 | pixels_per_port)
        multicore_fifo_push_blocking((uint32_t)info.pixel_data);
        multicore_fifo_push_blocking((info.num_ports << 16) | info.pixels_per_port);

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
