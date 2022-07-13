cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(classic-vga C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()
pico_enable_stdio_usb(hello_world 1) ①
pico_enable_stdio_uart(hello_world 0) 

add_subdirectory(src)