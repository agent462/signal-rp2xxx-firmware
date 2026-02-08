#ifndef STATUS_LED_H
#define STATUS_LED_H

#include <stdint.h>

typedef enum {
    STATUS_IDLE,       // blue solid -- waiting for first frame
    STATUS_RECEIVING,  // green blink -- frames arriving normally
    STATUS_CRC_ERROR,  // red blink -- CRC errors
    STATUS_TIMEOUT,    // red solid -- no frames for 2s
} status_state_t;

// configure GPIO4 (onboard NeoPixel) as output
void status_led_init(void);

// change the current status state
void status_led_set(status_state_t state);

// call per frame to drive blink toggle (flips every 4 frames ~5Hz at 40fps)
void status_led_update(uint32_t frame_count);

#endif
