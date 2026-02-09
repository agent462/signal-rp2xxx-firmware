cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico2_w)

include(../../common/pico_sdk_import.cmake)

project(signal_pico2w C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(signal_pico2w
    ../../common/main.c
    ../../common/spi_slave.c
    ../../common/ws281x_parallel.c
    ../../common/frame_protocol.c
    ../../common/bitplane.c
    ../../common/status_led.c
)

target_include_directories(signal_pico2w PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../common
    ${CMAKE_CURRENT_BINARY_DIR}
)

pico_generate_pio_header(signal_pico2w
    ${CMAKE_CURRENT_SOURCE_DIR}/../../common/ws281x_parallel.pio
)

target_link_libraries(signal_pico2w
    pico_stdlib
    hardware_spi
    hardware_dma
    hardware_pio
    hardware_gpio
    hardware_clocks
    hardware_timer
    hardware_watchdog
    pico_multicore
)

pico_enable_stdio_usb(signal_pico2w 1)
pico_enable_stdio_uart(signal_pico2w 0)
pico_add_extra_outputs(signal_pico2w)
