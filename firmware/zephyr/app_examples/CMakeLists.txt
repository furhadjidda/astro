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

# The ESP32 WiFi adapter (libzephyr) and picolibc both define random().
# Allow the linker to keep the first definition and ignore the duplicate.
if(CONFIG_WIFI_ESP32 AND CONFIG_PICOLIBC)
    zephyr_link_libraries(-Wl,--allow-multiple-definition)
endif()


target_sources(app PRIVATE examples/all_sensors_test.cpp)
message(board name is ${BOARD} \n\n)
if(BOARD STREQUAL "adafruit_feather_esp32s3")
    target_sources(app PRIVATE examples/status_class.cpp)
endif()
