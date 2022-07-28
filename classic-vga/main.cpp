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

typedef const struct timings {
  const uint32_t clock;
  const uint8_t hfront;
  const uint8_t hsync;
  const uint8_t hback;
  const uint16_t hactive;
  const uint8_t vfront;
  const uint8_t vsync;
  const uint8_t vback;
  const uint16_t vactive;
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

#define SCREEN_WIDTH 512
#define SCREEN_HEIGHT 342
#define SCREEN_BPP 1
#define STRIDE ((SCREEN_WIDTH/(8*sizeof(uint32_t))/SCREEN_BPP))
// 768 > 684(aka 342*2) so we have 84 extra scanlines
const uint32_t framebuffer_scanlines = xga_timings.vactive/2;
const uint32_t output_scanlines = xga_timings.vactive;
const uint32_t framebuffer_words = STRIDE*framebuffer_scanlines;
uint32_t framebuffer[framebuffer_words] ;
const void *framebuffer_p = framebuffer; // store a pointer to the framebuffer for DMA to read from
void *output_scanline_ptrs[output_scanlines];
const void *output_scanline_ptrs_p = output_scanline_ptrs;
volatile uint32_t *vidgen_sm_p = &pio0->txf[VIDEO_SM];
// somewhere to dump non-data from the dreq SM
// we only want to trigger, the data send is just 0
volatile uint32_t next_scanline_dump;

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
  vsync_input[0] = MIN0(timings.vfront, 1) << 16 | IRQ_SET_SIDE_1;
  vsync_input[1] = MIN0(timings.vsync, 1) << 16 | NOP_SIDE_0;
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
  printf("Actual frequency: %i\r\n", actual_frequency);

  pio_sm_set_clkdiv_int_frac(pio, sm, div256 / 256, div256 % 256);
}

// feed the same data into a FIFO constantly
void setup_looping_dma(uint32_t src_data[], volatile void *dest, uint len, uint dreq) {
  uint channel = dma_claim_unused_channel(true);
  uint restart_channel = dma_claim_unused_channel(true);

  printf("sync DMA setup: FIFO: %x DREQ: %i, DMA: %i restart DMA: %i\r\n", dest, dreq, channel, restart_channel);

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

// feed the frame buffer into a FIFO constantly
// double scan lines, and fill unused scanlines
void setup_framebuffer_dma(PIO pio, uint sm) {
  const uint channel = dma_claim_unused_channel(true);
  const uint loop_channel = dma_claim_unused_channel(true);
  const uint restart_channel = dma_claim_unused_channel(true);
  const uint next_scanline_channel = dma_claim_unused_channel(true);
  const uint next_scanline_restart_channel = dma_claim_unused_channel(true);
  const uint dreq = pio_get_dreq(pio, sm, true);
  const uint next_scanline_dreq = pio_get_dreq(pio, NEXT_SL_SM, false);;

  printf("vidgen DMA setup: FIFO: %x DREQ: %i, DMA: %i loop DMA: %i restart DMA: %i\r\n", vidgen_sm_p, dreq, channel, loop_channel, restart_channel);

  dma_channel_config c = dma_channel_get_default_config(channel);
  //channel_config_set_ring(&c, false, 6); // 64-byte (one scanline) ring
  //channel_config_set_high_priority(&c, true);
  channel_config_set_dreq(&c, dreq);
  channel_config_set_chain_to(&c, loop_channel);
  dma_channel_configure(
    channel,
    &c,
    vidgen_sm_p,
    NULL, // scanlines fed in by the loop DMA channel
    STRIDE, // one scanline
    false // started after the whole chain is setup
  );

  // loop over scanlines
  // set the start of each scanline and trigger the first DMA channel
  dma_channel_config loop_channel_c = dma_channel_get_default_config(loop_channel);
  channel_config_set_dreq(&loop_channel_c, next_scanline_dreq);
  channel_config_set_chain_to(&loop_channel_c, restart_channel);
  channel_config_set_read_increment(&loop_channel_c, true);
  channel_config_set_write_increment(&loop_channel_c, false);
  dma_channel_configure(
    loop_channel,
    &loop_channel_c,
    &dma_hw->ch[channel].al3_read_addr_trig,
    output_scanline_ptrs,
    output_scanlines,
    false
  );

  // reset the first DMA channel to the start of the frame buffer
  dma_channel_config restart_c = dma_channel_get_default_config(restart_channel);
  channel_config_set_read_increment(&restart_c, false);
  channel_config_set_write_increment(&restart_c, false);
  dma_channel_configure(
    restart_channel,
    &restart_c,
    &dma_hw->ch[loop_channel].al3_read_addr_trig,
    &output_scanline_ptrs_p,
    1,
    false
  );

  // read the input from the next_scanline PIO so it doesn't stall
  dma_channel_config next_scanline_channel_c = dma_channel_get_default_config(next_scanline_channel);
  channel_config_set_dreq(&next_scanline_channel_c, next_scanline_dreq);
  channel_config_set_chain_to(&next_scanline_channel_c, next_scanline_restart_channel);
  channel_config_set_read_increment(&next_scanline_channel_c, false);
  channel_config_set_write_increment(&next_scanline_channel_c, false);
  dma_channel_configure(
    next_scanline_channel,
    &next_scanline_channel_c,
    &next_scanline_dump,
    &pio->rxf[NEXT_SL_SM],
    0xffffff, // 0xffffffff doesn't work??
    true
  );

  dma_channel_config next_scanline_restart_channel_c = dma_channel_get_default_config(next_scanline_restart_channel);
  channel_config_set_read_increment(&next_scanline_restart_channel_c, false);
  channel_config_set_write_increment(&next_scanline_restart_channel_c, false);
  dma_channel_configure(
    next_scanline_restart_channel,
    &next_scanline_restart_channel_c,
    &dma_hw->ch[next_scanline_channel].al2_write_addr_trig,
    &dma_hw->ch[next_scanline_channel].write_addr,
    1,
    false
  );

  dma_channel_start(loop_channel);
}

// read everything into the framebuffer
void setup_capture_dma(PIO pio, uint sm) {
  uint channel = dma_claim_unused_channel(true);
  uint restart_channel = dma_claim_unused_channel(true);
  const volatile void *src = &pio->rxf[sm];
  const uint dreq = pio_get_dreq(pio, sm, false);
  printf("vidcap DMA setup: FIFO: %x DREQ: %i, DMA: %i restart DMA: %i\r\n", src, dreq, channel, restart_channel);

  dma_channel_config c = dma_channel_get_default_config(channel);
  channel_config_set_dreq(&c, dreq);
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, true);
  channel_config_set_chain_to(&c, restart_channel);
  //channel_config_set_enable(&c, true);
  dma_channel_configure(
    channel,
    &c,
    framebuffer,
    src,
    STRIDE*SCREEN_HEIGHT,
    false
  );
  

  // reset write_addr to the start of the framebuffer and trigger
  dma_channel_config restart_c = dma_channel_get_default_config(restart_channel);
  channel_config_set_read_increment(&restart_c, false);
  channel_config_set_write_increment(&restart_c, false);
  dma_channel_configure(
    restart_channel,
    &restart_c,
    &dma_hw->ch[channel].al2_write_addr_trig,
    &framebuffer_p,
    1,
    false
  );
  printf("initial vidcap DMA write: %x read %x transfer_cnt: %i ctrl: %x fbwords: %i\r\n", 
      dma_hw->ch[channel].write_addr, 
      dma_hw->ch[channel].read_addr,
      //dma_hw->ch[channel].transfer_count,
      dma_debug_hw->ch[channel].tcr,
      dma_hw->ch[channel].ctrl_trig,
      framebuffer_words
    );
  dma_channel_start(channel);
}

int main() {
  // overclock to 130MHz to run 1024x768@60 XGA
  set_sys_clock_khz(130000, true);
  stdio_init_all();

  const uint LED_PIN = 25;
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);


  printf("waiting for usb host");
  while (!tud_cdc_connected()) {
    printf(".");
    sleep_ms(500);
  }
  printf("\nusb host detected!\n");


  const PIO vidgen_pio = pio0;
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
  setup_framebuffer_dma(vidgen_pio, VIDEO_SM);

  printf("Generating test pattern. %i words, stride: %i words x%i scanlines\r\n", framebuffer_words, STRIDE, framebuffer_scanlines);
  // generate test pattern in frame buffer
  // leave the extra scanlines empty
  for (uint32_t i = 0; i < STRIDE*SCREEN_HEIGHT; i++) {
    //framebuffer[i] = 0xff0000ff;
    //framebuffer[i] = 0xffffffff;
    if ((i/STRIDE) % 2 == 0) {
      framebuffer[i] = 0xaaaaaaaa;
    } else {
      framebuffer[i] = 0x55555555;
    }
  }

  for (uint32_t i=STRIDE*SCREEN_HEIGHT; i < framebuffer_words; i++) {
    framebuffer[i] = 0;
  }
  // blank scanlines for "cool" CRT scanline effect
  //uint32_t blank_scanline[STRIDE] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

  for (uint i=0; i < framebuffer_scanlines; i++) {
    output_scanline_ptrs[i*2] = &framebuffer[STRIDE*i];
    output_scanline_ptrs[i*2+1] = &framebuffer[STRIDE*i];
  }

  // start the PIOs
  pio_sm_set_enabled(vidgen_pio, HSYNC_SM, true);
  pio_sm_set_enabled(vidgen_pio, VSYNC_SM, true);
  pio_sm_set_enabled(vidgen_pio, VIDEO_SM, true);
  pio_sm_set_enabled(vidgen_pio, NEXT_SL_SM, true);
  printf("output SMs started\r\n");

  // ====== Mac capture =====
  const PIO vidcap_pio = pio1;
  uint vidcap_sm = vidcap_program_init(vidcap_pio, 12);
  printf("vidcap program loaded on SM %i\r\n", vidcap_sm);
  // setup capture DMA
  setup_capture_dma(vidcap_pio, vidcap_sm);
  printf("vidcap DMA started\r\n");

  // start capturing
  pio_sm_set_enabled(vidcap_pio, vidcap_sm, true);
  printf("Capture SM started\r\n");

  while (true) {
    gpio_xor_mask(1<<LED_PIN);
    if (pio_sm_is_tx_fifo_empty(vidgen_pio, HSYNC_SM)) {
      printf("HSYNC starved at %i\r\n", pio_sm_get_pc(vidgen_pio, HSYNC_SM));
    }
    if (pio_sm_is_tx_fifo_empty(vidgen_pio, VSYNC_SM)) {
      printf("VSYNC starved at %i\r\n", pio_sm_get_pc(vidgen_pio, VSYNC_SM));
    }

    if (pio_sm_is_rx_fifo_full(vidcap_pio, vidcap_sm)) {
      printf("vidcap stalled at %i\r\n", pio_sm_get_pc(vidcap_pio, vidcap_sm));
    }
    //printf("vidcap RX FIFO: %i\r\n", pio_sm_get_rx_fifo_level(vidcap_pio, vidcap_sm));

    if (vidgen_pio->fdebug & PIO_FDEBUG_TXSTALL_BITS) {
      printf("vidgen stalled, clearing\r\n");
      vidgen_pio->fdebug = 1 << (PIO_FDEBUG_TXSTALL_LSB+2);
    }
#if 0
    printf("In: %i (PC: %i) Out: %x fdebug0: %x fdebug1: %x transfer_cnt: %i\r\n", 
      (uint32_t)dma_hw->ch[7].write_addr - (uint32_t)framebuffer, 
      pio_sm_get_pc(vidcap_pio, vidcap_sm),
      (uint32_t)dma_hw->ch[4].read_addr,
      vidgen_pio->fdebug,
      vidcap_pio->fdebug,
      dma_hw->ch[9].transfer_count
    );
#endif
    sleep_ms(1000);
  }
}



