#ifndef BITPLANE_H
#define BITPLANE_H

#include <stdint.h>
#include "pico/types.h"

// number of 32-bit words needed in the bitplane buffer
#define BITPLANE_WORDS(pixels_per_port) ((pixels_per_port) * 24)

// transform port-sequential pixel data into bitplane format for PIO output.
//
// pixel_data layout (port-sequential):
//   [port0_pix0_byte0, port0_pix0_byte1, port0_pix0_byte2,
//    port0_pix1_byte0, ...,
//    port1_pix0_byte0, ...]
//
// bitplane layout:
//   word[pixel * 24 + byte * 8 + bit] has bit N set if port N's
//   pixel has that bit HIGH. MSB first within each byte.
//
// color order is determined by the byte order in pixel_data
// (handled upstream by the Go daemon, not here).
//
// pixel_data: input pixel bytes, size = num_ports * pixels_per_port * 3
// bitplane: output buffer, size >= BITPLANE_WORDS(pixels_per_port) uint32_t's
// num_ports: number of LED strips (1-8)
// pixels_per_port: number of pixels per strip
void bitplane_transform(const uint8_t *pixel_data, uint32_t *bitplane,
                        uint num_ports, uint pixels_per_port);

// transform a subset of ports from a larger frame into a separate bitplane.
// extracts ports [port_offset .. port_offset+port_count) from the frame,
// remapping them to bit positions 0..port_count-1 in the output.
//
// pixel_data: full frame pixel bytes (all ports * pixels_per_port * 3)
// bitplane: output buffer, size >= BITPLANE_WORDS(pixels_per_port) uint32_t's
// port_offset: first port index to extract
// port_count: number of ports to extract (1-8)
// pixels_per_port: number of pixels per strip
void bitplane_transform_subset(const uint8_t *pixel_data, uint32_t *bitplane,
                               uint port_offset, uint port_count,
                               uint pixels_per_port);

#endif
