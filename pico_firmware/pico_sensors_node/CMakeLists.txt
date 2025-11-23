cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pico_sensors_node C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

include_directories("${ROOT_DIR_FIRMWARE}/pico_gnss_node/gnss_parser")
include_directories("${CMAKE_SOURCE_DIR}/pico_sensors_node/imu")
include_directories("${CMAKE_SOURCE_DIR}/pico_sensors_node/gnss")
include_directories("${CMAKE_SOURCE_DIR}/pico_sensors_node/tof")
include_directories("${CMAKE_SOURCE_DIR}/pico_sensors_node/display")
include_directories("${CMAKE_SOURCE_DIR}/pico_sensors_node")

link_directories(${MICROROS_ROOT}/libmicroros)

add_executable(pico_sensors_node
    sensor_node.cpp
    pico_uart_transport.c
    sensorFactory.cpp
    imu/imu.cpp
    gnss/gnss.cpp
    tof/tof.cpp
    display/display.cpp
)
target_link_libraries(pico_sensors_node
    pico_stdlib
    microros
    pico_cyw43_arch_none
    hardware_i2c
    Adafruit_Gps_Library
    gnss_parser
    bno055
    vl53l0x
    oled_driver_1306
    flash_manager
    pico_multicore
)

target_include_directories(pico_sensors_node PUBLIC
    ${MICROROS_ROOT}/libmicroros/include
    ${CMAKE_SOURCE_DIR}/pico_sensors_node/gnss_parser
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -mthumb")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -mthumb")
# Configure Pico
pico_enable_stdio_usb(pico_sensors_node 1)
pico_enable_stdio_uart(pico_sensors_node 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Generate UF2
pico_add_extra_outputs(pico_sensors_node)
