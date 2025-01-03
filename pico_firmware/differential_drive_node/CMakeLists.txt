cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(differential_drive_node C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

link_directories(${MICROROS_ROOT}/libmicroros)

add_executable(differential_drive_node
    differential_drive.cpp
    pico_uart_transport.c
    motor_control.cpp
    encoder.cpp
    odometry.cpp
    utility/median_filter.cpp
)
target_link_libraries(differential_drive_node
    pico_stdlib
    microros
    pico_cyw43_arch_none
    hardware_i2c
    pca9685
)

target_include_directories(differential_drive_node PUBLIC
    ${MICROROS_ROOT}/libmicroros/include
    utility
)

# Additional compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -mcpu=cortex-m0 -mthumb")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -mcpu=cortex-m0 -mthumb")
# Configure Pico
pico_enable_stdio_usb(differential_drive_node 1)
pico_enable_stdio_uart(differential_drive_node 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Generate UF2
pico_add_extra_outputs(differential_drive_node)


link_directories(${MICROROS_ROOT}/libmicroros)

add_executable(differential_drive_test
    test.cpp
    pico_uart_transport.c
    motor_control.cpp
    encoder.cpp
    odometry.cpp
    utility/median_filter.cpp
)
target_link_libraries(differential_drive_test
    pico_stdlib
    microros
    pico_cyw43_arch_none
    hardware_i2c
    pca9685
)

target_include_directories(differential_drive_test PUBLIC
    ${MICROROS_ROOT}/libmicroros/include
    utility
)

# Additional compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -mcpu=cortex-m0 -mthumb")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -mcpu=cortex-m0 -mthumb")
# Configure Pico
pico_enable_stdio_usb(differential_drive_test 1)
pico_enable_stdio_uart(differential_drive_test 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Generate UF2
pico_add_extra_outputs(differential_drive_test)