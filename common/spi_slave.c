#include "spi_slave.h"
#include "config.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/resets.h"
#include <stdio.h>
#include <string.h>

// max protocol overhead for buffer sizing:
// sync(2) + length(2) + frame_num(4) + flags(1) + crc(2) = 11
// phase 3 receives raw bytes; protocol parsing is added in phase 4.
#define PROTOCOL_OVERHEAD 11
#define RX_BUF_SIZE (MAX_FRAME_SIZE + PROTOCOL_OVERHEAD)

// double buffers: DMA fills one while main loop processes the other.
// NOTE: PL022 DREQ stays asserted after real data ends, causing DMA to
// overrun the buffer with zero reads at bus speed. frame_len will be
// RX_BUF_SIZE, not the actual frame size. use the protocol header's
// length field (phase 4) to determine actual frame boundaries.
static uint8_t rx_buf[2][RX_BUF_SIZE];
static volatile int active_buf = 0;
static volatile uint32_t frame_len = 0;
static volatile bool frame_avail = false;
static int dma_chan = -1;

// debug: ISR captures DMA remaining count for main loop to print
static volatile uint32_t debug_dma_remaining = 0;
static volatile uint32_t debug_fifo_extra = 0;

static void start_dma_receive(void);

// GPIO interrupt on CS rising edge -- fires when the SPI master finishes
// a transfer (deasserts chip select). RP2040 GPIO interrupts work
// regardless of pin function select, so this fires even though GPIO9
// is assigned to SPI1_CSn.
static void cs_rise_callback(uint gpio, uint32_t events) {
    if (gpio != PIN_SPI_CS || !(events & GPIO_IRQ_EDGE_RISE)) return;
    if (frame_avail) return;  // previous frame not consumed, drop this one

    // disable RXDMAE immediately to kill DREQ. after the last real byte,
    // the PL022 DREQ can stay asserted, causing DMA to race through the
    // entire buffer reading zeros at bus speed before this ISR fires.
    hw_clear_bits(&spi_get_hw(SPI_PORT)->dmacr, SPI_SSPDMACR_RXDMAE_BITS);

    // stop DMA and calculate how many bytes it transferred
    dma_channel_abort(dma_chan);
    uint32_t remaining = dma_channel_hw_addr(dma_chan)->transfer_count;
    uint32_t dma_bytes = RX_BUF_SIZE - remaining;

    // disable SSE to fully reset PL022 receive state.
    // prevents DREQ from reasserting when we re-arm DMA.
    hw_clear_bits(&spi_get_hw(SPI_PORT)->cr1, SPI_SSPCR1_SSE_BITS);

    // drain any bytes still sitting in the SPI RX FIFO.
    // DMA is paced by DREQ so a few trailing bytes may not have
    // been picked up before the abort.
    uint8_t *buf = rx_buf[active_buf];
    uint32_t extra = 0;
    while (spi_is_readable(SPI_PORT) && (dma_bytes + extra) < RX_BUF_SIZE) {
        buf[dma_bytes + extra] = (uint8_t)spi_get_hw(SPI_PORT)->dr;
        extra++;
    }

    debug_dma_remaining = remaining;
    debug_fifo_extra = extra;

    uint32_t total = dma_bytes + extra;
    if (total == 0) {
        // spurious CS edge with no data -- re-arm DMA so we don't
        // permanently stall reception
        start_dma_receive();
        return;
    }

    frame_len = total;
    frame_avail = true;
    gpio_put(PIN_READY, 0);  // tell master we're busy
}

void spi_slave_init(void) {
    // configure READY pin: output, start HIGH (ready for data)
    gpio_init(PIN_READY);
    gpio_set_dir(PIN_READY, GPIO_OUT);
    gpio_put(PIN_READY, 1);

    // reset SPI peripheral to clean state.
    // bypass spi_init() which briefly enables SSE with SCR=255/CPSR=254,
    // violating the PL022 slave requirement: f_SSPCLK >= f_SCK / 12.
    // with f_SSPCLK too slow, DREQ misbehaves (stays asserted, DMA reads
    // empty FIFO producing 19200 garbage bytes).
    reset_block(SPI_RESET_BITS);
    unreset_block_wait(SPI_RESET_BITS);

    // after reset, SSE=0 (peripheral disabled). configure everything first.
    // CR0: DSS=7 (8-bit), SPO=1 SPH=1 (Mode 3), SCR=0
    // PL022 slave REQUIRES CPHA=1 for multi-byte transfers with CS held low.
    // with CPHA=0, the slave shift register freezes after byte 1 (per ARM DDI0194).
    spi_get_hw(SPI_PORT)->cr0 = (7u << SPI_SSPCR0_DSS_LSB)
                              | SPI_SSPCR0_SPO_BITS
                              | SPI_SSPCR0_SPH_BITS;
    // CPSR=2: minimum prescaler -> f_SSPCLK = 180 MHz / (2 * 1) = 90 MHz
    spi_get_hw(SPI_PORT)->cpsr = 2;
    // CR1: slave mode (MS=1), SSE still disabled
    spi_get_hw(SPI_PORT)->cr1 = SPI_SSPCR1_MS_BITS;

    // connect GPIO pins to SPI1 peripheral.
    // pull resistors prevent noise reception when the Pi master isn't driving:
    //   CS pull-up   = deselected (HIGH) when idle
    //   SCK pull-up  = clock idles HIGH (CPOL=1, Mode 3)
    //   RX pull-down  = zeros if somehow clocked while idle
    gpio_set_function(PIN_SPI_SCK, GPIO_FUNC_SPI);
    gpio_pull_up(PIN_SPI_SCK);
    gpio_set_function(PIN_SPI_RX, GPIO_FUNC_SPI);
    gpio_pull_down(PIN_SPI_RX);
    gpio_set_function(PIN_SPI_TX, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_CS, GPIO_FUNC_SPI);
    gpio_pull_up(PIN_SPI_CS);

    // claim DMA channel for SPI RX.
    // SSE and RXDMAE are enabled by start_dma_receive() in the correct
    // order (SSE first, DMA configured, then RXDMAE) to prevent DREQ glitch.
    dma_chan = dma_claim_unused_channel(true);

    // debug: test DMA DREQ pacing before enabling interrupts.
    // start DMA, wait 100ms with no SPI activity, check if DMA waited.
    start_dma_receive();
    sleep_ms(100);
    uint32_t test_rem = dma_channel_hw_addr(dma_chan)->transfer_count;
    printf("DMA DREQ test: remaining=%lu/%d after 100ms idle (%s)\n",
           (unsigned long)test_rem, RX_BUF_SIZE,
           test_rem == RX_BUF_SIZE ? "DREQ working" : "DREQ BROKEN");
    // abort test DMA before restarting properly
    dma_channel_abort(dma_chan);

    // enable GPIO interrupt on CS rising edge for frame boundary detection.
    // note: gpio_set_irq_enabled_with_callback sets one callback per core.
    // if other modules need GPIO interrupts, centralize into a dispatcher.
    gpio_set_irq_enabled_with_callback(PIN_SPI_CS, GPIO_IRQ_EDGE_RISE,
                                       true, cs_rise_callback);

    // start DMA receive for real
    start_dma_receive();

    // debug: print DMA config for verification
    uint dreq = spi_get_dreq(SPI_PORT, false);
    uint32_t ctrl = dma_channel_hw_addr(dma_chan)->ctrl_trig;
    uint treq_sel = (ctrl >> 15) & 0x3f;
    printf("DMA ch=%d dreq=%d ctrl=0x%08lx treq_sel=%d\n",
           dma_chan, dreq, (unsigned long)ctrl, treq_sel);
}

static void start_dma_receive(void) {
    // drain any residual bytes from previous transfer
    while (spi_is_readable(SPI_PORT)) {
        (void)spi_get_hw(SPI_PORT)->dr;
    }

    // re-enable SSE (disabled in ISR to reset PL022 state)
    hw_set_bits(&spi_get_hw(SPI_PORT)->cr1, SPI_SSPCR1_SSE_BITS);

    dma_channel_config dc = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&dc, DMA_SIZE_8);
    channel_config_set_read_increment(&dc, false);   // always read from SPI DR
    channel_config_set_write_increment(&dc, true);    // increment into buffer
    channel_config_set_dreq(&dc, spi_get_dreq(SPI_PORT, false));  // RX DREQ

    dma_channel_configure(dma_chan, &dc,
        rx_buf[active_buf],            // write destination
        &spi_get_hw(SPI_PORT)->dr,     // read source (SPI data register)
        RX_BUF_SIZE,                   // max transfer count
        true                           // start immediately
    );

    // re-enable RXDMAE after DMA is configured and ready.
    // must be after SSE enable and DMA start to prevent DREQ glitch.
    hw_set_bits(&spi_get_hw(SPI_PORT)->dmacr, SPI_SSPDMACR_RXDMAE_BITS);
}

bool spi_slave_frame_ready(void) {
    return frame_avail;
}

bool spi_slave_get_frame(const uint8_t **data, uint32_t *len) {
    if (!frame_avail) return false;
    *data = rx_buf[active_buf];
    *len = frame_len;
    return true;
}

uint32_t spi_slave_debug_remaining(void) {
    return debug_dma_remaining;
}

uint32_t spi_slave_debug_fifo_extra(void) {
    return debug_fifo_extra;
}

void spi_slave_frame_consumed(void) {
    frame_avail = false;
    frame_len = 0;

    // swap to alternate buffer for next reception
    active_buf ^= 1;

    // restart DMA on fresh buffer
    start_dma_receive();

    // signal master we're ready for the next frame
    gpio_put(PIN_READY, 1);
}
