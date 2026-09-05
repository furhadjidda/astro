#pragma once

#include <rcl/rcl.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

/* =========================================================
 * Publisher period constants (milliseconds)
 * ========================================================= */

#define GNSS_PUBLISH_PERIOD_MS 1000
#define IMU_PUBLISH_PERIOD_MS 200
#define IIM42652_PUBLISH_PERIOD_MS 200
#define TIME_SYNC_PERIOD_MS 1000
#define LPS22HB_PUBLISH_PERIOD_MS 2000
#define OLED_VIEW_SWITCH_PERIOD_MS 5000

/* =========================================================
 * Executor configuration
 * ========================================================= */

#define EXECUTOR_STACK_SIZE 8192
#define EXECUTOR_PRIORITY 4

/* =========================================================
 * Error-handling macros
 * ========================================================= */

#define RCCHECK(fn)                                             \
    do {                                                        \
        rcl_ret_t rc = (fn);                                    \
        if (rc != RCL_RET_OK) {                                 \
            LOG_DBG("RCL error %d at line %d\n", rc, __LINE__); \
            for (;;) {                                          \
                k_sleep(K_FOREVER);                             \
            }                                                   \
        }                                                       \
    } while (0)

#define RCSOFTCHECK(fn)                                           \
    do {                                                          \
        rcl_ret_t rc = (fn);                                      \
        if (rc != RCL_RET_OK) {                                   \
            LOG_DBG("RCL warning %d at line %d\n", rc, __LINE__); \
        }                                                         \
    } while (0)
