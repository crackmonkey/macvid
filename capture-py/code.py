
# https://nerdhut.de/2016/06/26/macintosh-classic-crt-1/ was very helpful for timings

import time
import array
import board
import rp2pio
import adafruit_pioasm

screen_width = 512
screen_height = 342

# in pins: video, hsync, vsync
vidcap = adafruit_pioasm.Program(f"""
.program vidcap
.side_set 1

jmp pin acquireline ; vsync high (inactive), grab a scanline
vsync:
    wait 0 pin 2 ; wait for a vsync pulse
    wait 1 pin 2 ; wait out the vsync pulse
set x, 23
vback_porch: ; 24 lines
    wait 0 pin 1
    wait 1 pin 1
    jmp x-- vback_porch

acquireline:
wait 0 pin 1 ; wait for HSYNC low (start)
; load 178 into x, HBLANK time: HSYNC to start of data
; we should have plenty of time to do this in HBLANK
set x, 0b001
in x, 3
set x, 0b1011
in x, 4
in null, 24
mov x, isr
mov isr, null ; clear ISR for actual data
jmp x-- dec
dec:
wait 0 pin 1 ; wait for HSYNC low
acquireline_wait: ; hblank back porch
  jmp x-- acquireline_wait [7] ; 8 cycle (1px) wait

; pixloop_512 makes eight 64-bit acquisitions for a total of 512 pixels.
set y, 7 side 1
pixloop_512:
    ; 0x4 >> 24 = 64
    set x, 4 side 1
    in x, 4 side 1 
    in null, 24 side 1 
    mov x, isr side 1 
    mov isr, null side 1 ; clear ISR for actual data
    ; skip past the nops because the loop setup took those cycles for the first pixel
    jmp x-- pixloop_sample side 1 
    pixloop: ; 8 clocks
        nop side 1 [2]
    pixloop_sample:
        in pins, 1 side 1 [3]
        ;nop side 1 [3]
        jmp x-- pixloop side 1
    jmp y-- pixloop_512 side 1
""")

sm = rp2pio.StateMachine(
    vidcap.assembled,
    frequency=125_000_000,
    #frequency=10000,
    auto_push=True,
    push_threshold=32,
    jmp_pin=board.GP18, # vsync
    first_in_pin=board.GP16,
    in_pin_count=3,
    #in_shift_right=False,
    # Use the side_set for showing the timings
    first_sideset_pin=board.GP19,
    initial_sideset_pin_state=0,
    initial_sideset_pin_direction=1,
    **vidcap.pio_kwargs
)
print(f"real frequency {sm.frequency/1000000}Mhz aka {sm.frequency//8/1000000}MHz x8")
print(f'PIO Program length: {len(vidcap.assembled)}/32')

#framebuf = [
#  array.array('L', [0]*(screen_width*screen_height//32)),
#  array.array('L', [0]*(screen_width*screen_height//32))
#  ]
# a bytearray drops less frames
framebuf = [
  bytearray(screen_width*screen_height//8),
  bytearray(screen_width*screen_height//8)
  ]

stride = screen_width//32
bufnum = 0
error_frames = 0

while True:
    # Read a whole frame
    sm.readinto(framebuf[bufnum])
    bufnum = 1 if bufnum == 0 else 0

    # DMA Read a scanline at a time
        #print(framebuf[0:ready])
        #print("0x%08X" % framebuf[0], end=",")
        
    if sm.rxstall:
        sm.clear_rxfifo()
        #print(f"RX Stalled! {bufnum}")
        sm.restart()
        error_frames += 1
        if error_frames % 100 == 0:
            print(f'{error_frames} frames garbled')
    if sm.txstall:
        sm.clear_txstall()
        print("TX Stalled!")
    #time.sleep(1.0)