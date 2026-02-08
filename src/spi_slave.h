#ifndef SPI_SLAVE_H
#define SPI_SLAVE_H

#include <stdint.h>
#include <stdbool.h>

// initialize SPI1 as slave with DMA reception and READY pin.
// must be called after set_sys_clock_khz() -- PL022 slave needs 12x
// oversampling: f_SSPCLK = 180/(CPSR=2) = 90 MHz, max SCK = 7.5 MHz.
// uses SPI Mode 3 (CPOL=1, CPHA=1) -- Mode 0 limits PL022 slave to 1 byte.
void spi_slave_init(void);

// check if a complete frame has been received (CS deasserted after data).
// call this in the main loop.
bool spi_slave_frame_ready(void);

// get pointer to received frame data and its length.
// only valid after spi_slave_frame_ready() returns true.
// pointer remains valid until spi_slave_frame_consumed() is called.
bool spi_slave_get_frame(const uint8_t **data, uint32_t *len);

// mark the current frame as consumed and re-arm for next reception.
// swaps to the alternate buffer, restarts DMA, and sets READY HIGH.
void spi_slave_frame_consumed(void);

// debug: DMA remaining count from last ISR (0 = DMA completed fully before ISR)
uint32_t spi_slave_debug_remaining(void);

// debug: extra bytes drained from SPI FIFO after DMA abort
uint32_t spi_slave_debug_fifo_extra(void);

#endif
