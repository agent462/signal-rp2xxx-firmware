#ifndef FRAME_PROTOCOL_H
#define FRAME_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

// frame header layout (v2, 15 bytes):
// [0:1]     sync word 0xAA55
// [2:5]     data length (big-endian uint32, pixel data for active ports only)
// [6:9]     frame number (big-endian uint32, monotonic counter)
// [10]      flags: bits 7:4 = protocol version (2), bit 0 = DMX data follows
// [11:14]   port mask (big-endian uint32, bit i set = port i has pixel data)
// [15..]    pixel data (active ports only, lowest bit first in mask order)
// if flags bit 0 set:
//   [15 .. 15+P-1]        pixel data (P bytes, stride-aligned)
//   [15+P .. 15+P+D-1]    DMX channel data (D bytes, 1-512)
//   [15+P+D .. 15+P+D+1]  DMX data length D (big-endian, 2 bytes)
// [end-1 : end] CRC-16-CCITT (big-endian)
//
// CRC covers bytes 2 through end-2 (LENGTH + FRAME_NUM + FLAGS + PORT_MASK + all DATA).
// sync word is excluded (known constant adds no validation value).
//
// port mask: only ports with changed pixel data are included in the frame.
// firmware maintains a persistent pixel_state buffer; unchanged ports keep
// their last data. active_ports = popcount(port_mask).

#define SYNC_WORD              0xAA55
#define FRAME_HEADER_SIZE      15 // sync(2) + length(4) + frame_num(4) + flags(1) + port_mask(4)
#define FRAME_CRC_SIZE         2
#define FRAME_PROTOCOL_VERSION 2
#define FRAME_FLAG_DMX         0x01  // flags bit 0: DMX data follows pixel data

typedef enum {
    FRAME_OK = 0,
    FRAME_ERR_TOO_SHORT,   // fewer bytes than minimum frame (header + CRC)
    FRAME_ERR_BAD_SYNC,    // sync word mismatch
    FRAME_ERR_BAD_LENGTH,  // data length exceeds buffer or max frame size
    FRAME_ERR_BAD_PORTS,   // port mask invalid (zero or bits beyond board's port count)
    FRAME_ERR_BAD_CRC,     // CRC-16 mismatch
    FRAME_ERR_ALIGNMENT,   // data length not divisible by (active_ports * 3)
    FRAME_ERR_VERSION,     // protocol version mismatch
} frame_result_t;

typedef struct {
    uint32_t frame_num;        // monotonic frame counter from sender
    uint32_t data_len;         // pixel data length in bytes (active ports only)
    uint32_t port_mask;        // bitmask of ports with pixel data in this frame
    uint32_t active_ports;     // popcount of port_mask
    uint32_t pixels_per_port;  // derived: data_len / (active_ports * 3)
    const uint8_t *pixel_data; // pointer into raw buffer at pixel data start
#ifdef PIN_DMX_TX
    bool dmx_present;          // true if DMX data follows pixel data
    const uint8_t *dmx_data;   // DMX channel data (after pixel data)
    uint32_t dmx_len;          // DMX data length (1-512)
#endif
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
