#include <stdio.h>
#include <math.h>

#include "pico.h"
#include "pico/multicore.h"
#include "pico/sync.h"
#include "tusb.h"

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
// Our assembled program:
#include "capture.pio.h"
#include "vgaout.pio.h"

#define MIN0(x, dec) (x-dec>0 ? (uint16_t)roundl(x-dec) : 0)

#define SCREEN_WIDTH 512
#define SCREEN_HEIGHT 342
#define SCREEN_BPP 1
#define STRIDE (SCREEN_WIDTH/(32/SCREEN_BPP))

uint32_t framebuffer[STRIDE*SCREEN_HEIGHT];
uint32_t framebuffer_size = sizeof(framebuffer);

typedef const struct timings {
  uint32_t clock;
  uint8_t hfront;
  uint8_t hsync;
  uint8_t hback;
  uint16_t hactive;
  uint8_t vfront;
  uint8_t vsync;
  uint8_t vback;
  uint16_t vactive;
} timings_t;

const timings_t mac_mode = {
  .clock = 15667200,
  .hfront = 14,
  .hsync = 178,
  .hback = 0,
  .hactive = 512,
  .vfront = 0,
  .vsync = 4,
  .vback = 24,
  .vactive = 342
};

const timings_t xga_timings = {
  .clock = 65000000,
  .hfront = 24,
  .hsync = 136,
  .hback = 160,
  .hactive = 1024,
  .vfront = 3,
  .vsync = 6,
  .vback = 29,
  .vactive = 768
};

// sync programs take pairs of counters and setup instrutions for each
// stage of their main loop
// front, sync, back, active
// aligned (16) to be able to use the DMA ring buffer feature
uint32_t hsync_input[4] __attribute__ ((aligned (16)));
uint32_t vsync_input[4] __attribute__ ((aligned (16)));

// fill in looping delay inputs for the sync state machines
void calc_timing_inputs(timings_t &timings) {
  float sys_clock_s = (1.0/clock_get_hz(clk_sys));
  float pixel_s = (1.0/timings.clock);
  float pixel_clk = pixel_s / sys_clock_s;

  hsync_input[0] = MIN0(timings.hfront * pixel_clk, 4) << 16 | NOP_SIDE_1;
  hsync_input[1] = MIN0(timings.hsync * pixel_clk, 4) << 16 | NOP_SIDE_0;
  hsync_input[2] = MIN0(timings.hback * pixel_clk, 4) << 16 | NOP_SIDE_1;
  hsync_input[3] = MIN0(timings.hactive * pixel_clk, 4) << 16 | IRQ_CLEAR_SIDE_1;

  // vsyncs are all defined in terms of scanlines
  vsync_input[0] = MIN0(timings.vfront, 1) << 16 | NOP_SIDE_1;
  vsync_input[1] = MIN0(timings.vsync, 1) << 16 | IRQ_SET_SIDE_0;
  vsync_input[2] = MIN0(timings.vback, 1) << 16 | NOP_SIDE_1;
  vsync_input[3] = MIN0(timings.vactive, 1) << 16 | IRQ_CLEAR_SIDE_1;
}

// calculate the exact 16:8 clock divider for the target_freq
// ?? is this worth doing over using the float version?
void set_pixel_clock(uint32_t target_freq, PIO pio, uint sm) {
  uint64_t frequency256 = ((uint64_t)clock_get_hz(clk_sys)) * 256;
  uint64_t div256 = frequency256 / (2*target_freq);
  if (frequency256 % div256 > 0) {
      div256 += 1;
  }
  uint32_t actual_frequency = frequency256 / div256;
  printf("Actual frequency: %i\n", actual_frequency);

  pio_sm_set_clkdiv_int_frac(pio, sm, div256 / 256, div256 % 256);
}

// feed the same data into a FIFO constantly
void setup_looping_dma(uint32_t src_data[], volatile void *dest, uint len, uint dreq) {
  uint channel = dma_claim_unused_channel(true);
  uint restart_channel = dma_claim_unused_channel(true);

  dma_channel_config c = dma_channel_get_default_config(channel);
  channel_config_set_dreq(&c, dreq);
  channel_config_set_chain_to(&c, restart_channel);
  channel_config_set_ring(&c, false, 4);
  dma_channel_configure(
    channel,
    &c,
    dest,
    src_data,
    len,
    true
  );

  // write the write_addr back as a trigger
  // set_ring makes the read_addr loop around so we don't need to reset it
  dma_channel_config restart_c = dma_channel_get_default_config(restart_channel);
  channel_config_set_read_increment(&restart_c, false);
  channel_config_set_write_increment(&restart_c, false);
  dma_channel_configure(
    restart_channel,
    &restart_c,
    &dma_hw->ch[channel].al2_write_addr_trig,
    &dma_hw->ch[channel].write_addr,
    1,
    false
  );

}
 
int main() {
  stdio_init_all();

/*
  printf("waiting for usb host");
  while (!tud_cdc_connected()) {
    printf(".");
    sleep_ms(500);
  }
  printf("\nusb host detected!\n");
*/

  // overclock to 130MHz to run 1024x768@60 XGA
  set_sys_clock_khz(130000,true);
  setup_default_uart();

  PIO vidgen_pio = pio0;
  vidgen_program_init(vidgen_pio, 16, 17, 18);
  set_pixel_clock(xga_timings.clock, vidgen_pio, VIDEO_SM);
  calc_timing_inputs(xga_timings);

  // setup DMA to feed *sync_inputs to their respective SMs
  uint dreq;
  volatile void *tx_fifo;
  // hsync
  dreq = pio_get_dreq(vidgen_pio, HSYNC_SM, true);
  tx_fifo = &vidgen_pio->txf[HSYNC_SM];
  setup_looping_dma(hsync_input, tx_fifo, sizeof(hsync_input), dreq);
  // vsync
  dreq = pio_get_dreq(vidgen_pio, VSYNC_SM, true);
  tx_fifo = &vidgen_pio->txf[VSYNC_SM];
  setup_looping_dma(vsync_input, tx_fifo, sizeof(vsync_input), dreq);

  // setup DMA to feed the frame buffer to the video signal SM
  dreq = pio_get_dreq(vidgen_pio, VIDEO_SM, true);
  tx_fifo = &vidgen_pio->txf[VIDEO_SM];
  setup_looping_dma(framebuffer, tx_fifo, sizeof(framebuffer), dreq);

  // generate test pattern in frame buffer
  for (uint32_t i = 0; i < framebuffer_size; i++) {
    framebuffer[i] = 0xff0000ff;
  }

  // start the PIOs
  pio_sm_set_enabled(vidgen_pio, HSYNC_SM, true);
  pio_sm_set_enabled(vidgen_pio, VSYNC_SM, true);
  pio_sm_set_enabled(vidgen_pio, VIDEO_SM, true);

  const uint LED_PIN = 25;
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);

  while (true) {
    gpio_xor_mask(1<<LED_PIN);
    if (pio_sm_is_tx_fifo_empty(vidgen_pio, HSYNC_SM)) {
      printf("HSYNC Stalled at %i\r\n", pio_sm_get_pc(vidgen_pio, HSYNC_SM));
    }
    if (pio_sm_is_tx_fifo_empty(vidgen_pio, VSYNC_SM)) {
      printf("VSYNC Stalled at %i\r\n", pio_sm_get_pc(vidgen_pio, VSYNC_SM));
    }

    sleep_ms(1000);
  }
}



