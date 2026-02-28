#include "config.h"
#include "frame_protocol.h"
#include <stddef.h>

uint16_t crc16_ccitt(const uint8_t *data, uint32_t len) {
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

frame_result_t frame_parse(const uint8_t *raw_data, uint32_t raw_len,
                           frame_info_t *info) {
    if (raw_len < FRAME_HEADER_SIZE + FRAME_CRC_SIZE)
        return FRAME_ERR_TOO_SHORT;

    // sync word (SYNC_WORD = 0xAA55, big-endian)
    if (raw_data[0] != (SYNC_WORD >> 8) || raw_data[1] != (SYNC_WORD & 0xFF))
        return FRAME_ERR_BAD_SYNC;

    // payload data length (big-endian uint32) -- covers pixel data + optional DMX
    uint32_t data_len = ((uint32_t)raw_data[2] << 24)
                      | ((uint32_t)raw_data[3] << 16)
                      | ((uint32_t)raw_data[4] << 8)
                      | raw_data[5];

    // total frame size must fit in the received buffer
    uint32_t total = FRAME_HEADER_SIZE + data_len + FRAME_CRC_SIZE;
    if (total > raw_len || data_len > MAX_FRAME_SIZE)
        return FRAME_ERR_BAD_LENGTH;

    // port count from flags bits 3:0
    uint32_t flags = raw_data[10];
    uint32_t ports = flags & 0x0F;
#ifdef TOTAL_PORTS
    if (ports == 0 || ports > TOTAL_PORTS)
#else
    if (ports == 0 || ports > NUM_PORTS)
#endif
        return FRAME_ERR_BAD_PORTS;

    uint32_t stride = ports * 3;

    // determine pixel data length (may be less than data_len if DMX present)
    uint32_t pixel_data_len = data_len;

#ifdef PIN_DMX_TX
    uint32_t dmx_len = 0;
    const uint8_t *dmx_data_ptr = NULL;
    bool dmx_present = false;

    if (flags & FRAME_FLAG_DMX_BIT) {
        // DMX layout within payload: [pixel_data(P)][dmx_data(D)][dmx_length(2)]
        // dmx_length is at the last 2 bytes of the payload, so it can be
        // located without knowing the pixel/DMX split point.
        if (data_len < stride + 3)
            return FRAME_ERR_BAD_LENGTH;  // not enough room for pixels + DMX

        // read dmx_length from end of payload
        const uint8_t *payload = raw_data + FRAME_HEADER_SIZE;
        dmx_len = ((uint32_t)payload[data_len - 2] << 8) | payload[data_len - 1];

        if (dmx_len == 0 || dmx_len > 512)
            return FRAME_ERR_BAD_LENGTH;

        if (dmx_len + 2 >= data_len)
            return FRAME_ERR_BAD_LENGTH;

        // pixel_bytes = total payload - dmx_data - dmx_length field
        uint32_t pixel_bytes = data_len - dmx_len - 2;
        if (pixel_bytes == 0 || (pixel_bytes % stride) != 0)
            return FRAME_ERR_BAD_LENGTH;

        pixel_data_len = pixel_bytes;
        dmx_data_ptr = payload + pixel_bytes;
        dmx_present = true;
    }
#endif

    // pixel data must divide evenly into (ports * 3 bytes/pixel)
    if (pixel_data_len == 0 || (pixel_data_len % stride) != 0)
        return FRAME_ERR_ALIGNMENT;

    uint32_t pixels_per_port = pixel_data_len / stride;
    if (pixels_per_port > MAX_PIXELS)
        return FRAME_ERR_BAD_LENGTH;

    // CRC-16 over everything between sync and CRC:
    // LENGTH(4) + FRAME_NUM(4) + FLAGS(1) + DATA(N) = 9 + data_len bytes
    const uint8_t *crc_start = raw_data + 2;
    uint32_t crc_len = 9 + data_len;
    uint16_t computed = crc16_ccitt(crc_start, crc_len);
    uint16_t received = ((uint16_t)raw_data[FRAME_HEADER_SIZE + data_len] << 8)
                      | raw_data[FRAME_HEADER_SIZE + data_len + 1];

    if (computed != received)
        return FRAME_ERR_BAD_CRC;

    info->frame_num = ((uint32_t)raw_data[6] << 24)
                    | ((uint32_t)raw_data[7] << 16)
                    | ((uint32_t)raw_data[8] << 8)
                    | raw_data[9];
    info->data_len = pixel_data_len;
    info->num_ports = ports;
    info->pixels_per_port = pixels_per_port;
    info->pixel_data = raw_data + FRAME_HEADER_SIZE;

#ifdef PIN_DMX_TX
    info->dmx_present = dmx_present;
    info->dmx_data = dmx_data_ptr;
    info->dmx_len = dmx_len;
#endif

    return FRAME_OK;
}
