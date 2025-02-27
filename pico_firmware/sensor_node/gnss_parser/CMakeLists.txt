cmake_minimum_required(VERSION 3.12)

project(pico_gnss_sensor_node C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
add_library(gnss_parser
    gnss_parser.cpp
)

target_include_directories(gnss_parser PUBLIC
    ${MICROROS_ROOT}/libmicroros/include
    ${ROOT_DIR_FIRMWARE}/external/micro_ros_raspberrypi_pico_sdk/libmicroros/include
)

target_link_libraries(gnss_parser
    pico_stdlib
    microros
    pico_cyw43_arch_none
)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra ")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra ")