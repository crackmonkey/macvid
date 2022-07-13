add_executable(classic-vga)

pico_generate_pio_header(capture_pio ${CMAKE_CURRENT_LIST_DIR}/capture.pio)

target_sources(capture_pio PRIVATE main.cpp)

# pull in common dependencies
target_link_libraries(capture_pio PRIVATE
    pico_stdlib
    hardware_pio)

# create map/bin/hex file etc.
pico_add_extra_outputs(classic-vga)

