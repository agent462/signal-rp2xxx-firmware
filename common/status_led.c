// status_led.c -- onboard NeoPixel status indicator
//
// bitbang WS2812 on PIN_NEOPIXEL (onboard RGB LED on SCORPIO/Adafruit boards).
// updates only on state transitions or blink toggle to minimize interrupt
// blackout time (~90us per 3-byte send).
//
// if PIN_NEOPIXEL is not defined in config.h, all functions compile to no-ops.
// custom boards that lack an onboard NeoPixel simply omit the define.

#include "status_led.h"
#include "config.h"

#ifdef PIN_NEOPIXEL

#include "pico/stdlib.h"
#include "pico/platform.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"

// WS2812 timing at 180 MHz (1 cycle = 5.56ns)
// T0H: 350ns = 63 cycles, T0L: 800ns = 144 cycles
// T1H: 700ns = 126 cycles, T1L: 600ns = 108 cycles
#define T0H_CYCLES 63
#define T0L_CYCLES 144
#define T1H_CYCLES 126
#define T1L_CYCLES 108

// dim brightness -- onboard NeoPixel is very bright
#define DIM 16

// blink: toggle every 4 frames (~5Hz at 40fps)
#define BLINK_INTERVAL 4

// GRB color values for each state (dimmed)
static const uint8_t state_colors[][3] = {
    [STATUS_IDLE]      = {0,   0,   DIM},  // blue: GRB = (0, 0, 16)
    [STATUS_RECEIVING] = {DIM, 0,   0},    // green: GRB = (16, 0, 0)
    [STATUS_CRC_ERROR] = {0,   DIM, 0},    // red: GRB = (0, 16, 0)
    [STATUS_TIMEOUT]   = {0,   DIM, 0},    // red: GRB = (0, 16, 0)
};

static status_state_t current_state = STATUS_IDLE;
static bool blink_on = true;
static bool needs_update = true;

// send a single bit to the NeoPixel via bitbang.
// must run from RAM to avoid XIP cache miss jitter.
static void __not_in_flash_func(send_bit)(bool bit) {
    if (bit) {
        gpio_put(PIN_NEOPIXEL, 1);
        busy_wait_at_least_cycles(T1H_CYCLES);
        gpio_put(PIN_NEOPIXEL, 0);
        busy_wait_at_least_cycles(T1L_CYCLES);
    } else {
        gpio_put(PIN_NEOPIXEL, 1);
        busy_wait_at_least_cycles(T0H_CYCLES);
        gpio_put(PIN_NEOPIXEL, 0);
        busy_wait_at_least_cycles(T0L_CYCLES);
    }
}

// send 3 bytes (GRB) to the NeoPixel with interrupts disabled.
// total blackout: ~90us (24 bits * ~3.75us per bit, well within tolerance).
static void __not_in_flash_func(send_pixel)(uint8_t g, uint8_t r, uint8_t b) {
    uint32_t irq_state = save_and_disable_interrupts();

    uint8_t bytes[3] = {g, r, b};
    for (int i = 0; i < 3; i++) {
        for (int bit = 7; bit >= 0; bit--) {
            send_bit((bytes[i] >> bit) & 1);
        }
    }

    restore_interrupts(irq_state);
}

void status_led_init(void) {
    gpio_init(PIN_NEOPIXEL);
    gpio_set_dir(PIN_NEOPIXEL, GPIO_OUT);
    gpio_put(PIN_NEOPIXEL, 0);

    // WS2812 reset: hold LOW for >280us
    sleep_us(300);

    current_state = STATUS_IDLE;
    blink_on = true;
    needs_update = true;
}

void status_led_set(status_state_t state) {
    if (state == current_state) {
        return;
    }
    current_state = state;
    blink_on = true;
    needs_update = true;
}

void status_led_update(uint32_t frame_count) {
    bool is_blink = (current_state == STATUS_RECEIVING || current_state == STATUS_CRC_ERROR);

    if (is_blink && (frame_count % BLINK_INTERVAL == 0)) {
        blink_on = !blink_on;
        needs_update = true;
    }

    if (!needs_update) {
        return;
    }
    needs_update = false;

    // determine if pixel should be on or off
    bool show = !is_blink || blink_on;

    if (show) {
        const uint8_t *c = state_colors[current_state];
        send_pixel(c[0], c[1], c[2]);
    } else {
        send_pixel(0, 0, 0);
    }
}

#else // PIN_NEOPIXEL not defined -- no-op stubs

void status_led_init(void) {}
void status_led_set(status_state_t state) { (void)state; }
void status_led_update(uint32_t frame_count) { (void)frame_count; }

#endif
