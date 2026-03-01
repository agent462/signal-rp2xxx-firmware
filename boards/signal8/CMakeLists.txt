cmake_minimum_required(VERSION 3.13)

# RP2350A on custom Signal 8 carrier board (not a Pico 2 W).
# Using pico2 board target for RP2350 SDK definitions.
set(PICO_BOARD pico2)

include(../../common/pico_sdk_import.cmake)

project(signal_8 C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(signal_8
    ../../common/main.c
    ../../common/spi_slave.c
    ../../common/ws281x_parallel.c
    ../../common/frame_protocol.c
    ../../common/bitplane.c
    ../../common/status_led.c
    ../../common/dmx_output.c
)

target_include_directories(signal_8 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../common
    ${CMAKE_CURRENT_BINARY_DIR}
)

target_compile_definitions(signal_8 PRIVATE
    PICO_CLOCK_ADJUST_PERI_CLOCK_WITH_SYS_CLOCK=1
)

pico_generate_pio_header(signal_8
    ${CMAKE_CURRENT_SOURCE_DIR}/../../common/ws281x_parallel.pio
)

pico_generate_pio_header(signal_8
    ${CMAKE_CURRENT_SOURCE_DIR}/../../common/dmx_output.pio
)

target_link_libraries(signal_8
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

pico_enable_stdio_usb(signal_8 1)
pico_enable_stdio_uart(signal_8 0)
pico_add_extra_outputs(signal_8)
