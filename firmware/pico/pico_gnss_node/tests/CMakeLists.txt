cmake_minimum_required(VERSION 3.14)

set(PROJECT_NAME "gnss_sensor_node_test")
set(BUILD_NUMBER "1")

link_directories(${CMAKE_SOURCE_DIR}/gnss_node_firmware/gnss_parser)
# Enable testing
enable_testing()

# Add a simple test executable
add_executable(${PROJECT_NAME} 
    test_main.cpp
    gnss_sensor_node_test.cpp
    ${ROOT_DIR_FIRMWARE}/gnss_node_firmware/gnss_parser/gnss_parser.cpp
)


# Link test libraries
target_link_libraries(${PROJECT_NAME}
    gtest
    gtest_main
    Adafruit_Gps_Library
)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${MICROROS_ROOT}/libmicroros/include
    ${ROOT_DIR_FIRMWARE}/gnss_node_firmware/gnss_parser
    ${ROOT_DIR_FIRMWARE}/external/micro_ros_raspberrypi_pico_sdk/libmicroros/include
)

# Add tests
add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})


message(STATUS "Configuring for host platform complete !!")
