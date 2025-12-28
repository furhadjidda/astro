# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/modules/libmicroros)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(zephyr_microros_sensor_node)
if(CONFIG_PICOLIBC)
    add_compile_definitions(__STDC_WANT_LIB_EXT1__=1)
endif()
target_sources(app PRIVATE src/main.cpp src/status_class.cpp)
