#include "bitplane.h"
#include <string.h>

void bitplane_transform(const uint8_t *pixel_data, uint32_t *bitplane,
                        uint num_ports, uint pixels_per_port) {
    uint total_words = pixels_per_port * 24;
    memset(bitplane, 0, total_words * sizeof(uint32_t));

    uint bytes_per_port = pixels_per_port * 3;

    for (uint port = 0; port < num_ports; port++) {
        const uint8_t *port_data = pixel_data + port * bytes_per_port;
        uint port_bit = 1u << port;

        for (uint pixel = 0; pixel < pixels_per_port; pixel++) {
            const uint8_t *pix = port_data + pixel * 3;
            uint base_word = pixel * 24;

            // 3 bytes per pixel, MSB first within each byte
            for (uint byte_idx = 0; byte_idx < 3; byte_idx++) {
                uint8_t val = pix[byte_idx];
                if (val == 0) continue;  // skip zero bytes (common in sparse frames)

                uint word_offset = base_word + byte_idx * 8;
                for (uint bit = 0; bit < 8; bit++) {
                    if (val & (0x80 >> bit)) {
                        bitplane[word_offset + bit] |= port_bit;
                    }
                }
            }
        }
    }
}

void bitplane_transform_subset(const uint8_t *pixel_data, uint32_t *bitplane,
                               uint port_offset, uint port_count,
                               uint pixels_per_port) {
    uint total_words = pixels_per_port * 24;
    memset(bitplane, 0, total_words * sizeof(uint32_t));

    uint bytes_per_port = pixels_per_port * 3;

    for (uint i = 0; i < port_count; i++) {
        // index into full frame at the correct port
        const uint8_t *port_data = pixel_data + (port_offset + i) * bytes_per_port;
        uint port_bit = 1u << i;  // remap to bit 0..port_count-1

        for (uint pixel = 0; pixel < pixels_per_port; pixel++) {
            const uint8_t *pix = port_data + pixel * 3;
            uint base_word = pixel * 24;

            for (uint byte_idx = 0; byte_idx < 3; byte_idx++) {
                uint8_t val = pix[byte_idx];
                if (val == 0) continue;

                uint word_offset = base_word + byte_idx * 8;
                for (uint bit = 0; bit < 8; bit++) {
                    if (val & (0x80 >> bit)) {
                        bitplane[word_offset + bit] |= port_bit;
                    }
                }
            }
        }
    }
}
