set(ASTRO_MODULES_DIR "${CMAKE_CURRENT_LIST_DIR}")
list(APPEND ZEPHYR_EXTRA_MODULES
    "${ASTRO_MODULES_DIR}/libraries/micro_ros"
    "${ASTRO_MODULES_DIR}/drivers/sensors/bno055"
    "${ASTRO_MODULES_DIR}/drivers/gnss"
)
