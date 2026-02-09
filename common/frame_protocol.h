#ifndef FRAME_PROTOCOL_H
#define FRAME_PROTOCOL_H

#include <stdint.h>

// frame header layout:
// [0:1]  sync word 0xAA55
// [2:3]  data length (big-endian, bytes of pixel data only)
// [4:7]  frame number (big-endian, monotonic counter)
// [8]    flags: bits 3:0 = port count (1-8)
// [9..]  pixel data (port-sequential: all port0 pixels, then port1, etc.)
// [9+N : 9+N+1] CRC-16-CCITT (big-endian)
//
// CRC covers bytes 2 through 8+N (LENGTH + FRAME_NUM + FLAGS + DATA).
// sync word is excluded (known constant adds no validation value).

#define SYNC_WORD         0xAA55
#define FRAME_HEADER_SIZE 9  // sync(2) + length(2) + frame_num(4) + flags(1)
#define FRAME_CRC_SIZE    2

typedef enum {
    FRAME_OK = 0,
    FRAME_ERR_TOO_SHORT,   // fewer bytes than minimum frame (header + CRC)
    FRAME_ERR_BAD_SYNC,    // sync word mismatch
    FRAME_ERR_BAD_LENGTH,  // data length exceeds buffer or max frame size
    FRAME_ERR_BAD_PORTS,   // port count 0 or > NUM_PORTS
    FRAME_ERR_BAD_CRC,     // CRC-16 mismatch
    FRAME_ERR_ALIGNMENT,   // data length not divisible by (ports * 3)
} frame_result_t;

typedef struct {
    uint32_t frame_num;       // monotonic frame counter from sender
    uint32_t data_len;        // pixel data length in bytes
    uint32_t num_ports;       // LED port count (from flags bits 3:0)
    uint32_t pixels_per_port; // derived: data_len / (num_ports * 3)
    const uint8_t *pixel_data; // pointer into raw buffer at pixel data start
} frame_info_t;

// parse and validate a received SPI frame.
// raw_data: pointer to raw DMA buffer (may contain trailing zeros from DREQ overrun)
// raw_len: buffer length (may be larger than actual frame due to overrun)
// info: populated on FRAME_OK, undefined on error
frame_result_t frame_parse(const uint8_t *raw_data, uint32_t raw_len,
                           frame_info_t *info);

// compute CRC-16-CCITT (polynomial 0x1021, init 0xFFFF)
uint16_t crc16_ccitt(const uint8_t *data, uint32_t len);

#endif
