cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)
include(pico_extras_import_optional.cmake)

project(rgb_tasker C CXX ASM)
set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(
	rgb_tasker
	rgb_tasker.c
)

pico_generate_pio_header(rgb_tasker ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)

# pull in common dependencies and additional pwm hardware support
target_link_libraries(
	rgb_tasker
	pico_stdlib
	pico_multicore
	hardware_pio
	hardware_dma
	hardware_gpio
	pico_bootsel_via_double_reset
)

pico_enable_stdio_usb(rgb_tasker 1)
pico_enable_stdio_uart(rgb_tasker 0)

target_compile_definitions(
	rgb_tasker PRIVATE
	PICO_BOOTSEL_VIA_DOUBLE_RESET_ACTIVITY_LED=25
)

# create map/bin/hex file etc.
pico_add_extra_outputs(rgb_tasker)
