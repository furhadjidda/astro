cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(imu_gnss_node C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

include_directories("${ROOT_DIR_FIRMWARE}/gnss_node_firmware/gnss_parser")

link_directories(${MICROROS_ROOT}/libmicroros)

add_executable(imu_gnss_node
    imu_gnss_node.cpp
    pico_uart_transport.c
)
target_link_libraries(imu_gnss_node
    pico_stdlib
    microros
    pico_cyw43_arch_none
    hardware_i2c
    Adafruit_Gps_Library
    gnss_parser
    bno055
)

target_include_directories(imu_gnss_node PUBLIC
    ${MICROROS_ROOT}/libmicroros/include
    ${CMAKE_SOURCE_DIR}/gnss_node_firmware/gnss_parser
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -mcpu=cortex-m0 -mthumb")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -mcpu=cortex-m0 -mthumb")
# Configure Pico
pico_enable_stdio_usb(imu_gnss_node 1)
pico_enable_stdio_uart(imu_gnss_node 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Generate UF2
pico_add_extra_outputs(imu_gnss_node)
