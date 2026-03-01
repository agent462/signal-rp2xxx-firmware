cmake_minimum_required(VERSION 3.13)

# PICO_BOARD not set -- defaults to "pico" (standard RP2040).
# SCORPIO is an Adafruit Feather RP2040 but all pin config is in config.h,
# so the generic pico board target works with any pico-sdk installation.

include(../../common/pico_sdk_import.cmake)

project(signal_rp2040 C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(signal_rp2040
    ../../common/main.c
    ../../common/spi_slave.c
    ../../common/ws281x_parallel.c
    ../../common/frame_protocol.c
    ../../common/bitplane.c
    ../../common/status_led.c
)

target_include_directories(signal_rp2040 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../common
    ${CMAKE_CURRENT_BINARY_DIR}
)

target_compile_definitions(signal_rp2040 PRIVATE
    PICO_CLOCK_ADJUST_PERI_CLOCK_WITH_SYS_CLOCK=1
)

pico_generate_pio_header(signal_rp2040
    ${CMAKE_CURRENT_SOURCE_DIR}/../../common/ws281x_parallel.pio
)

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

pico_enable_stdio_usb(signal_rp2040 1)
pico_enable_stdio_uart(signal_rp2040 0)
pico_add_extra_outputs(signal_rp2040)
