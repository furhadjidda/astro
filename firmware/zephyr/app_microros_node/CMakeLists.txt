# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

get_filename_component(PARENT_DIR "${CMAKE_CURRENT_SOURCE_DIR}" DIRECTORY)
list(APPEND ZEPHYR_EXTRA_MODULES ${PARENT_DIR}/astro_custom_modules/libmicroros)
list(APPEND ZEPHYR_EXTRA_MODULES ${PARENT_DIR}/astro_custom_modules/sensor)
list(APPEND ZEPHYR_EXTRA_MODULES ${PARENT_DIR}/astro_custom_modules/iim42652_rak12033)
list(APPEND ZEPHYR_EXTRA_MODULES ${PARENT_DIR}/astro_custom_modules/gnss)

# Include local credentials (Wi-Fi SSID/password, agent IP) without committing them.
# Copy local.conf.example -> local.conf and fill in your values.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/local.conf")
    list(APPEND OVERLAY_CONFIG "${CMAKE_CURRENT_SOURCE_DIR}/local.conf")
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(microros_node)
if(CONFIG_PICOLIBC)
    add_compile_definitions(__STDC_WANT_LIB_EXT1__=1)
endif()

# The ESP32 WiFi adapter (libzephyr) and picolibc both define random().
# Allow the linker to pick the first definition (ESP32's) and ignore the duplicate.
if(CONFIG_WIFI_ESP32 AND CONFIG_PICOLIBC)
    zephyr_link_libraries(-Wl,--allow-multiple-definition)
endif()


target_sources(app PRIVATE src/node_main.cpp src/node_callbacks.cpp src/wifi_handler.cpp src/storage.cpp src/oled_wrapper.cpp src/oled_layout.cpp src/cfb_font_0608.c)
message(board name is ${BOARD} \n\n)
if(BOARD STREQUAL "adafruit_feather_esp32s3")
    target_sources(app PRIVATE src/status_class.cpp)
endif()
