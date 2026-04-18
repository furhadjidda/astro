cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pico_bno055_node C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

link_directories(${MICROROS_ROOT}/libmicroros)

add_executable(pico_bno055_node
    bno055_node.cpp
    pico_uart_transport.c
)
target_link_libraries(pico_bno055_node
    pico_stdlib
    microros
    pico_cyw43_arch_none
    hardware_i2c
    bno055
)

target_include_directories(pico_bno055_node PUBLIC
    ${MICROROS_ROOT}/libmicroros/include
)

# Additional compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -mthumb")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -mthumb")
# Configure Pico
pico_enable_stdio_usb(pico_bno055_node 1)
pico_enable_stdio_uart(pico_bno055_node 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Generate UF2
pico_add_extra_outputs(pico_bno055_node)
