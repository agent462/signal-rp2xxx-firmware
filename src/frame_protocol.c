#include "frame_protocol.h"
#include "config.h"

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

    // sync word
    if (raw_data[0] != 0xAA || raw_data[1] != 0x55)
        return FRAME_ERR_BAD_SYNC;

    // pixel data length (big-endian)
    uint32_t data_len = ((uint32_t)raw_data[2] << 8) | raw_data[3];

    // total frame size must fit in the received buffer
    uint32_t total = FRAME_HEADER_SIZE + data_len + FRAME_CRC_SIZE;
    if (total > raw_len || data_len > MAX_FRAME_SIZE)
        return FRAME_ERR_BAD_LENGTH;

    // port count from flags bits 3:0
    uint32_t ports = raw_data[8] & 0x0F;
    if (ports == 0 || ports > NUM_PORTS)
        return FRAME_ERR_BAD_PORTS;

    // pixel data must divide evenly into (ports * 3 bytes/pixel)
    uint32_t stride = ports * 3;
    if (data_len == 0 || (data_len % stride) != 0)
        return FRAME_ERR_ALIGNMENT;

    uint32_t pixels_per_port = data_len / stride;
    if (pixels_per_port > MAX_PIXELS)
        return FRAME_ERR_BAD_LENGTH;

    // CRC-16 over everything between sync and CRC:
    // LENGTH(2) + FRAME_NUM(4) + FLAGS(1) + DATA(N) = 7 + N bytes
    const uint8_t *crc_start = raw_data + 2;
    uint32_t crc_len = 7 + data_len;
    uint16_t computed = crc16_ccitt(crc_start, crc_len);
    uint16_t received = ((uint16_t)raw_data[FRAME_HEADER_SIZE + data_len] << 8)
                      | raw_data[FRAME_HEADER_SIZE + data_len + 1];

    if (computed != received)
        return FRAME_ERR_BAD_CRC;

    info->frame_num = ((uint32_t)raw_data[4] << 24)
                    | ((uint32_t)raw_data[5] << 16)
                    | ((uint32_t)raw_data[6] << 8)
                    | raw_data[7];
    info->data_len = data_len;
    info->num_ports = ports;
    info->pixels_per_port = pixels_per_port;
    info->pixel_data = raw_data + FRAME_HEADER_SIZE;

    return FRAME_OK;
}
