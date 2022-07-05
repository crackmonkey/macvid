# Output generator
# https://nerdhut.de/2016/06/26/macintosh-classic-crt-1/ was very helpful for timings

import time
import array
import board
import rp2pio
import adafruit_pioasm

screen_width = 512
screen_height = 342
stride = screen_width//32

# HSYNC & VSYNC need to be sequential
HSYNC_PIN = board.GP14
VSYNC_PIN = board.GP15
VIDEO_PIN = board.GP16

# in pins: hsync, vsync  out pin: video
vidgen = adafruit_pioasm.Program(f"""
.program vidcap
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

# generate hsync using IRQs and a second PIO SM
# clock at pixel clock
# Set IRQ for VSYNC code
fake_hsync = adafruit_pioasm.Program(f"""
.program fake_hsync
.side_set 1
; hblank = 192 pixels = 12x16 cycle delays
; hsync pulse is 288 pixels though (18x16) 18.45us
start_hsync:
.wrap_target
irq set 4
set x, 16 [14] ; irq set + set x + 14 waits = first 16 cycle delay
hsync:
    jmp x-- hsync [15]
nop ; +2 cycles because of clock inaccuracy
; video active period = 512 pixels = 16x32 cycles
; but hsync impinges 110 clocks into the video
; 402 = 18 + (12 * 32) pixels
; 413.75 for the timing to be right
set x, 11 side 1 [15]
nop side 1 [14]
hactive:
    nop side 1 [15]
    jmp x-- hactive  side 1 [15]
.wrap
""")
#fake_hsync.print_c_program("fake_hsync")

fake_vsync = adafruit_pioasm.Program(f"""
.program fake_vsync
.side_set 1
public fake_vsync:
; load 342 (0b101010110) into y
set y, 0b10101 side 1
in y, 5 side 1
set y, 0b0101 side 1
in y, 4 side 1
mov y, isr side 1
push noblock side 1
jmp y--, start_frame side 1 ; decriment y
.wrap_target
start_frame:
    mov x, y side 1
    scanline:
        wait 1 irq 4 side 1
        jmp x-- scanline side 1
    set x, 3 side 1 ; VSYNC pulse is 4 lines
    vsync:
        wait 1 irq 4 side 0
        jmp x--, vsync side 0
    set x, 23 ; 24 lines side 1
    vback_porch: 
        wait 1 irq 4 side 1
        jmp x-- vback_porch side 1
    .wrap
""", build_debuginfo=True)
#fake_vsync.print_c_program("fake_vsync")

# CircuitPython merges programs where it can
sm_hsync = rp2pio.StateMachine(
    fake_hsync.assembled,
    frequency=15625000,
    auto_push=True,
    first_sideset_pin=HSYNC_PIN,
    initial_sideset_pin_state=0,
    initial_sideset_pin_direction=1,
    **fake_hsync.pio_kwargs
)
print(f"hsync real frequency {sm_hsync.frequency/1000000}Mhz")

sm_vsync = rp2pio.StateMachine(
    fake_vsync.assembled,
    frequency=125_000_000//8, # embrace the clock skew
    #frequency=10000,
    auto_push=True,
    in_shift_right=False,
    first_sideset_pin=VSYNC_PIN,
    initial_sideset_pin_state=0,
    initial_sideset_pin_direction=1,
    **fake_vsync.pio_kwargs
)
print(f"vsync real frequency {sm_vsync.frequency/1000000}Mhz")
print(f'syncs Program length: {len(fake_vsync.assembled)+len(fake_hsync.assembled)}/32')

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
print(f"real frequency {sm.frequency/1000000}Mhz aka {sm.frequency//8/1000000}MHz x8")
print(f'PIO Program length: {len(vidgen.assembled)}/32')

framebuf = [
  array.array('L', [0]*(screen_width*screen_height//32)),
  array.array('L', [0]*(screen_width*screen_height//32))
  ]

bufnum = 0
error_frames = 0

# fill frame buffer
for i in range(0, screen_width*screen_height/32):
    framebuf[0][i] = 0xff0000ff # 16 / 16 off
    #framebuf[0][i] = 0xffff0000 if i % 2 == 0 else 0x00000000
#print(framebuf[0])

print('Generating test video signal')
# output a whole frame over and over
sm.background_write(loop=framebuf[bufnum])
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
    #time.sleep(1.0)