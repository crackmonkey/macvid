cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

# Pico_extras for scanvideo
include(../pico-extras/external/pico_extras_import.cmake)

project(classic-vga C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(classic-vga)

pico_generate_pio_header(classic-vga ${CMAKE_CURRENT_LIST_DIR}/capture.pio)
pico_generate_pio_header(classic-vga ${CMAKE_CURRENT_LIST_DIR}/vgaout.pio)

target_sources(classic-vga PRIVATE main.cpp)

# pull in common dependencies
target_link_libraries(classic-vga PRIVATE
    pico_multicore
    pico_stdlib
    hardware_pio
    hardware_dma
    )

pico_enable_stdio_usb(classic-vga 1)
pico_enable_stdio_uart(classic-vga 0) 

# create map/bin/hex file etc.
pico_add_extra_outputs(classic-vga)

