cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(signal_rp2040 C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(signal_rp2040
    src/main.c
    src/ws281x_parallel.c
    src/bitplane.c
    src/spi_slave.c
    src/frame_protocol.c
    src/status_led.c
)

# generate PIO header from .pio assembly
pico_generate_pio_header(signal_rp2040 ${CMAKE_CURRENT_LIST_DIR}/src/ws281x_parallel.pio)

target_include_directories(signal_rp2040 PRIVATE src ${CMAKE_CURRENT_BINARY_DIR})

target_link_libraries(signal_rp2040
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

# enable USB serial output, disable UART
pico_enable_stdio_usb(signal_rp2040 1)
pico_enable_stdio_uart(signal_rp2040 0)

pico_add_extra_outputs(signal_rp2040)
