# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

get_filename_component(PARENT_DIR "${CMAKE_CURRENT_SOURCE_DIR}" DIRECTORY)
include("${PARENT_DIR}/astro_custom_modules/modules.cmake")
list(APPEND ZEPHYR_EXTRA_MODULES "${ASTRO_MODULES_DIR}/drivers/sensors/iim42652")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(example_apps)
if(CONFIG_PICOLIBC)
    add_compile_definitions(__STDC_WANT_LIB_EXT1__=1)
endif()


target_sources(app PRIVATE src/filesystem_example.cpp)
message(board name is ${BOARD} \n\n)
