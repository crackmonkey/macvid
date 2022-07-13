# Output generator
# https://nerdhut.de/2016/06/26/macintosh-classic-crt-1/ was very helpful for timings

import array
import board
import rp2pio
import adafruit_pioasm

screen_width = 512
screen_height = 342
stride = screen_width//32

mac_mode = {
    "clock": 15667200,
    "hfront": 14,
    "hsync": 178,
    "hback": 0,
    "hactive": 512,
    "vfront": 0,
    "vsync": 4,
    "vback": 24,
    "vactive": 342,
}

output_mode = {
    "clock": 65000000,
    "hfront": 24,
    "hsync": 136,
    "hback": 160,
    "hactive": 1024,
    "vfront": 3,
    "vsync": 6,
    "vback": 29,
    "vactive": 768,
}

# calculate wall times and convert pixel clock counts into PIO clock counts
def calc_mode(timings, pio_clock):
  sys_clock_ns = (1/pio_clock) * 1E9
  px = 1/timings["clock"] * 1E9
  timings["pixel_ns"] = px
  timings["pixel_clk"] = px / sys_clock_ns
  timings['htot'] = timings['hfront'] + timings['hsync'] + timings['hback'] + timings['hactive']
  timings['vtot'] = timings['vfront'] + timings['vsync'] + timings['vback'] + timings['vactive']
  h_vals = [k for k in timings.keys() if k.startswith('h')]
  for k in h_vals:
    timings[k+'_ns'] = px * timings[k]
    timings[k+'_clk'] = px * timings[k] / sys_clock_ns
  v_vals = [k for k in timings.keys() if k.startswith('v')]
  for k in v_vals:
    timings[k+'_ns'] = timings['htot_ns'] * timings[k]
    timings[k+'_clk'] = timings['htot_clk'] * timings[k]
  return timings

output_timing = calc_mode(mac_mode, 125000000)
print(output_timing)

# HSYNC & VSYNC need to be sequential
HSYNC_PIN = board.GP14
VSYNC_PIN = board.GP15
VIDEO_PIN = board.GP16

# assemble these instructions to inject into the sync loops via the FIFO
sync_ops = adafruit_pioasm.Program(f"""
.program sync_ops
.side_set 1 opt
irq_set_pin_0:
irq set 4 rel side 0
irq_set_pin_1:
irq set 4 rel side 1
nop_pin_0:
irq clear 4 rel side 0
nop_pin_1:
irq clear 4 rel side 1
""", build_debuginfo=True)
#sync_ops.print_c_program("sync_ops")
IRQ_SET_PIN_0 = sync_ops.assembled[0]
IRQ_SET_PIN_1 = sync_ops.assembled[1]
IRQ_CLEAR_PIN_0 = sync_ops.assembled[2]
IRQ_CLEAR_PIN_1 = sync_ops.assembled[3]

# in pins: hsync, vsync  out pin: video
vidgen = adafruit_pioasm.Program(f"""
.program vidgen
.side_set 1
.wrap_target
jmp pin output_line ; vsync high (inactive), grab a scanline
vsync:
    wait 0 pin 1 ; wait for a vsync pulse
    wait 1 pin 1 ; wait out the vsync pulse
set x, 23
vback_porch: ; 24 lines
    wait 0 pin 0
    wait 1 pin 0
    jmp x-- vback_porch

output_line:
; load 178 into isr, HBLANK time: HSYNC to start of data
; 31 bits so it doesn't trigger auto push
set x, 0b001
in x, 3
set x, 0b1011
in x, 4
in null, 24
mov x, isr
mov isr, null ; clear ISR
jmp x-- dec
dec:
wait 0 pin 0 ; wait for HSYNC low (start)
hback_porch: ; hblank back porch
  jmp x-- hback_porch [7] ; 8 cycle (1px) wait

; pixloop_512 outputs eight 64-bit blocks for a total of 512 pixels.
; ??? needed??? : in between groups of 64-pixels add a 1-cycle delay to account for clock skew
set y, 7 side 1
pixloop_512:
    out pins, 1 side 1; output first pin while we setup the loop
    ; 0x4 >> 24 = 64
    set x, 4 side 1
    in x, 4 side 1 
    in null, 24 side 1 
    mov x, isr side 1 
    mov isr, null side 1 ; clear ISR for actual data
    jmp x-- dec64 side 1
    dec64:
    jmp x-- pixloop side 1 ; decrement x because we output the first pixel above
    pixloop: ; 8 clocks
        out pins, 1 side 1 [6]
        jmp x-- pixloop side 1
    jmp y-- pixloop_512 side 1
set pins, 0
.wrap
""")

# Arm code feeds in delay loop start value and a single setup intruction (read & execute = 2 clocks)
hsync_prog = """
.side_set 1 opt
out x, 16 ; get loop count
out exec, 16 ; run setup instruction
delay_loop:
    jmp x-- delay_loop
"""
gen_hsync = adafruit_pioasm.Program(hsync_prog)
#gen_hsync.print_c_program("gen_hsync")

gen_vsync = adafruit_pioasm.Program(f"""
.program gen_vsync
.side_set 1 opt
out x, 16 ; get loop count
out exec, 16 ; run setup instruction
delay_loop:
    wait 1 irq 4
    jmp x-- delay_loop
""", build_debuginfo=True)
#gen_vsync.print_c_program("gen_vsync")

# CircuitPython merges programs where it can
sm_hsync = rp2pio.StateMachine(
    gen_hsync.assembled,
    frequency=125_000_000,
    auto_pull=True,
    pull_threshold=16,
    first_sideset_pin=HSYNC_PIN,
    initial_sideset_pin_state=1,
    initial_sideset_pin_direction=1,
    **gen_hsync.pio_kwargs
)
#print(f"hsync real frequency {sm_hsync.frequency/1000000}Mhz")
# Delay loops (16 bit), setup instruction (16 bit)
# -3 clks for loop setup and -1 for the 0-based loop counter
hsync_timings = array.array('H', [
    round(output_timing['hfront_clk']) - 4, IRQ_CLEAR_PIN_1,
    round(output_timing['hsync_clk']) - 4, IRQ_SET_PIN_0,
    #round(output_timing['hback_clk']), IRQ_CLEAR_PIN_1,
    round(output_timing['hactive_clk']) - 4, IRQ_CLEAR_PIN_1,
])
#print(hsync_timings)
sm_hsync.background_write(loop=hsync_timings)

sm_vsync = rp2pio.StateMachine(
    gen_vsync.assembled,
    frequency=125_000_000,
    auto_pull=True,
    pull_threshold=16,
    first_sideset_pin=VSYNC_PIN,
    initial_sideset_pin_state=1,
    initial_sideset_pin_direction=1,
    **gen_vsync.pio_kwargs
)
#print(f"vsync real frequency {sm_vsync.frequency/1000000}Mhz")
print(f'combined syncs Program length: {len(gen_vsync.assembled)+len(gen_hsync.assembled)}/32')
# scanline loops (16 bit), setup instruction (16 bit)
vsync_timings = array.array('H', [
    #round(output_timing['vfront']), IRQ_CLEAR_PIN_1,
    round(output_timing['vsync']) - 1, IRQ_SET_PIN_0,
    round(output_timing['vback']) - 1, IRQ_CLEAR_PIN_1,
    round(output_timing['vactive']) - 1, IRQ_CLEAR_PIN_1,
])
sm_vsync.background_write(loop=vsync_timings)

sm = rp2pio.StateMachine(
    vidgen.assembled,
    frequency=125_000_000,
    auto_push=True,
    auto_pull=True,
    jmp_pin=board.GP18, # vsync
    first_in_pin=board.GP17,
    in_pin_count=2,
    first_out_pin=VIDEO_PIN,
    out_pin_count=1,
    first_set_pin=VIDEO_PIN,
    set_pin_count=1,
    # Use the side_set for showing the timings
    first_sideset_pin=board.GP13,
    initial_sideset_pin_state=0,
    initial_sideset_pin_direction=1,
    **vidgen.pio_kwargs
)
print(f"vidgen real frequency {sm.frequency/1000000}Mhz aka {sm.frequency//8/1000000}MHz x8")
print(f'vidgen Program length: {len(vidgen.assembled)}/32')

framebuf = [
  array.array('L', [0]*(screen_width*screen_height//32))
  ]

bufnum = 0
error_frames = 0

# fill frame buffer
for i in range(0, screen_width*screen_height/32):
    framebuf[0][i] = 0xff0000ff # 16 / 16 off

print('Generating test video signal')
# output a whole frame over and over
sm.background_write(loop=framebuf[0])
while True:
    if sm.rxstall:
        sm.clear_rxfifo()
        #print(f"RX Stalled! {bufnum}")
        
    if sm.txstall:
        sm.clear_txstall()
        #print("TX Stalled!")
        error_frames += 1
        if error_frames % 100 == 0:
            print(f'{error_frames} frames garbled')
        sm.restart()