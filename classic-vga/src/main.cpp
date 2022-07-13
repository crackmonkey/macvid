#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
// Our assembled program:
#include "capture.pio.h"

int main() {
   stdio_init_all();

  PIO capture_pio = pio0;
  uint offset = pio_add_program(capture_pio, &capture_program);
  uint sm = pio_claim_unused_sm(pio, true);
  capture_program_init(capture_pio, sm, offset, PICO_DEFAULT_LED_PIN);
}