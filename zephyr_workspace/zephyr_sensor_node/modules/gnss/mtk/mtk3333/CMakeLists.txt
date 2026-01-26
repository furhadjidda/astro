zephyr_library()

zephyr_library_sources(
    mtk3333.c
    nmea_parser.c
)

zephyr_library_include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
)
