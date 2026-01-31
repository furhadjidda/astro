/*
 *   This file is part of astro.
 *
 *   astro is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   astro is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License v3.0
 *   along with astro.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <bno055.h>  // Required for custom SENSOR_CHAN_*
#include <string.h>
#include <version.h>
#include <zephyr/device.h>
#include <zephyr/display/cfb.h>
#include <zephyr/drivers/gnss.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#if ZEPHYR_VERSION_CODE >= ZEPHYR_VERSION(3, 1, 0)
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#else
#include <sys/printk.h>
#include <zephyr.h>
#endif

#include <microros_transports.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>

LOG_MODULE_REGISTER(all_sensors_module, LOG_LEVEL_DBG);
// Display parameters
#define MAX_FONTS 42
#define SELECTED_FONT_INDEX 0
static uint16_t rows = 0;
static uint8_t ppt = 0;
static uint8_t font_width = 0;
static uint8_t font_height = 0;

// display driver
static const struct device* display_dev = DEVICE_DT_GET(DT_NODELABEL(ssd1306));
// imu driver
static const struct device* const bno055_dev = DEVICE_DT_GET(DT_NODELABEL(bno055));
// gnss driver
#define GNSS_MODEM DEVICE_DT_GET(DT_ALIAS(gnss))

// IMU configuration
static bool bno055_fusion = true;
#define BNO055_TIMING_STARTUP 400

/* =========================================================
 * Thread Configuration
 * ========================================================= */

#define THREAD_STACK_SIZE 4096
#define IMU_THREAD_PRIORITY 5

#define EXECUTOR_STACK_SIZE 4096
#define TIME_SYNC_STACK_SIZE 1024

#define EXECUTOR_PRIORITY 5
#define TIME_SYNC_PRIORITY 7 /* lower priority */

#define PUBLISH_PERIOD_MS 1000
#define TIME_SYNC_PERIOD_MS 1000

/* =========================================================
 * Error handling macros
 * ========================================================= */

#define RCCHECK(fn)                                            \
    do {                                                       \
        rcl_ret_t rc = (fn);                                   \
        if (rc != RCL_RET_OK) {                                \
            printk("RCL error %d at line %d\n", rc, __LINE__); \
            for (;;) {                                         \
                k_sleep(K_FOREVER);                            \
            }                                                  \
        }                                                      \
    } while (0)

#define RCSOFTCHECK(fn)                                          \
    do {                                                         \
        rcl_ret_t rc = (fn);                                     \
        if (rc != RCL_RET_OK) {                                  \
            printk("RCL warning %d at line %d\n", rc, __LINE__); \
        }                                                        \
    } while (0)

/* =========================================================
 * micro-ROS objects
 * ========================================================= */

static rclc_support_t support;
static rcl_node_t node;
static rcl_publisher_t publisher;
static rcl_timer_t timer;
static rclc_executor_t executor;

static std_msgs__msg__Int32 msg;

/* =========================================================
 * Thread objects
 * ========================================================= */

K_THREAD_STACK_DEFINE(imu_stack, THREAD_STACK_SIZE);
static struct k_thread imu_thread;

K_THREAD_STACK_DEFINE(executor_stack, EXECUTOR_STACK_SIZE);
static struct k_thread executor_thread;

K_THREAD_STACK_DEFINE(time_sync_stack, TIME_SYNC_STACK_SIZE);
static struct k_thread time_sync_thread;

/* =========================================================
 * Timer callback (runs inside executor thread)
 * ========================================================= */

static void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
    ARG_UNUSED(last_call_time);

    if (timer != NULL) {
        RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
        msg.data++;
    }
}

/* =========================================================
 * micro-ROS executor thread
 * ========================================================= */

static void executor_thread_entry(void* a, void* b, void* c) {
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        k_sleep(K_MSEC(1));
    }
}

/* =========================================================
 * Time synchronization thread
 * ========================================================= */

static void time_sync_thread_entry(void* a, void* b, void* c) {
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    /* Give transport + agent time to come up */
    k_sleep(K_SECONDS(2));
    while (1) {
        bool ok = rmw_uros_sync_session(50); /* 50 ms timeout */

        if (!ok) {
            printk("micro-ROS time sync failed\n");
        }
        k_sleep(K_MSEC(TIME_SYNC_PERIOD_MS));
    }
}

static void imu_thread_entry(void* a, void* b, void* c) {
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

#if Z_DEVICE_DT_FLAGS(DT_NODELABEL(bno055)) & DEVICE_FLAG_INIT_DEFERRED
    LOG_DBG("Deferred init enabled, sleeping for %d ms", BNO055_TIMING_STARTUP);
    k_sleep(K_MSEC(BNO055_TIMING_STARTUP));
    device_init(bno055_dev);
#endif
    if (!device_is_ready(bno055_dev)) {
        LOG_ERR("Device %s is not ready\n", bno055_dev->name);
        return;
    }
    LOG_DBG("BNO055 Device %s is ready\n", bno055_dev->name);

    while (1) {
        if (NULL != bno055_dev) {
            char buffer[64];
            struct sensor_value eul[3];

            sensor_sample_fetch(bno055_dev);
            sensor_channel_get(bno055_dev, static_cast<sensor_channel>(BNO055_SENSOR_CHAN_EULER_YRP), eul);
            LOG_DBG(
                "EULER: X(rad.s-1)[%d.%06d] Y(rad.s-1)[%d.%06d] "
                "Z(rad.s-1)[%d.%06d]\n",
                eul[0].val1, eul[0].val2, eul[1].val1, eul[1].val2, eul[2].val1, eul[2].val2);
            // Format for the screen

            snprintf(buffer, sizeof(buffer), "X=%d.%02d", eul[0].val1, eul[0].val2);
            cfb_print(display_dev, buffer, 0, 20);

            snprintf(buffer, sizeof(buffer), "Y=%d.%02d", eul[1].val1, eul[1].val2);
            cfb_print(display_dev, buffer, 0, 35);

            snprintf(buffer, sizeof(buffer), "Z=%d.%02d", eul[2].val1, eul[2].val2);
            cfb_print(display_dev, buffer, 0, 50);

            cfb_framebuffer_finalize(display_dev);

            struct sensor_value sys_status[3];
            sensor_channel_get(bno055_dev, static_cast<sensor_channel>(BNO055_SENSOR_CHAN_SYSTEM_STATUS), sys_status);
            LOG_DBG("system status %d , self_test = %d system_error %d \n", sys_status[0].val1, sys_status[1].val1,
                    sys_status[2].val1);

        } else {
            LOG_ERR("BNO055 device is NULL!\n");
        }
        k_sleep(K_MSEC(200));
    }
}

static void gnss_data_cb(const struct device* dev, const struct gnss_data* data) {
    uint64_t timepulse_ns;
    k_ticks_t timepulse;

    if (data->info.fix_status != GNSS_FIX_STATUS_NO_FIX) {
        if (gnss_get_latest_timepulse(dev, &timepulse) == 0) {
            timepulse_ns = k_ticks_to_ns_near64(timepulse);
        }
    }
    if (data->info.fix_status == GNSS_FIX_STATUS_GNSS_FIX) {
        char buffer[64];
        LOG_DBG("UTC Time: %02d %02d:%02d", data->utc.month, data->utc.hour, data->utc.minute);
        snprintf(buffer, sizeof(buffer), "Time:%02d:%02d", data->utc.hour, data->utc.minute);

        cfb_print(display_dev, buffer, 0, 0);  // Print at
        cfb_framebuffer_finalize(display_dev);
    }
}
GNSS_DATA_CALLBACK_DEFINE(GNSS_MODEM, gnss_data_cb);

#if CONFIG_GNSS_SATELLITES
static void gnss_satellites_cb(const struct device* dev, const struct gnss_satellite* satellites, uint16_t size) {
    unsigned int tracked_count = 0;
    unsigned int corrected_count = 0;

    for (unsigned int i = 0; i != size; ++i) {
        tracked_count += satellites[i].is_tracked;
        corrected_count += satellites[i].is_corrected;
    }
    printk("%u satellite%s reported (of which %u tracked, of which %u has RTK corrections)!\n", size,
           size > 1 ? "s" : "", tracked_count, corrected_count);
}
#endif
GNSS_SATELLITES_CALLBACK_DEFINE(GNSS_MODEM, gnss_satellites_cb);

#define GNSS_SYSTEMS_PRINTF(define, supported, enabled)                                                     \
    printk("\t%20s: Supported: %3s Enabled: %3s\n", STRINGIFY(define), (supported & define) ? "Yes" : "No", \
           (enabled & define) ? "Yes" : "No");

/* =========================================================
 * main()
 * ========================================================= */

int main(void) {
    // Starting display
    if (!device_is_ready(display_dev)) {
        LOG_ERR("Display device not ready\n");
        return -ENODEV;
    }

    if (display_set_pixel_format(display_dev, PIXEL_FORMAT_MONO01) != 0) {
        LOG_ERR("Failed to set required pixel format");
        return -ENOTSUP;
    }

    if (cfb_framebuffer_init(display_dev)) {
        LOG_ERR("Framebuffer init failed\n");
        return -EIO;
    }

    cfb_framebuffer_clear(display_dev, true);

    display_blanking_off(display_dev);
    display_set_orientation(display_dev, DISPLAY_ORIENTATION_ROTATED_180);

    rows = cfb_get_display_parameter(display_dev, CFB_DISPLAY_ROWS);
    ppt = cfb_get_display_parameter(display_dev, CFB_DISPLAY_PPT);

    for (int idx = 0; idx < MAX_FONTS; idx++) {
        if (cfb_get_font_size(display_dev, idx, &font_width, &font_height)) {
            break;  // end of font list, so exit loop.
        }

        LOG_DBG("index[%d] font width %d, font height %d", idx, font_width, font_height);
    }

    cfb_framebuffer_set_font(display_dev, SELECTED_FONT_INDEX);

    cfb_framebuffer_invert(display_dev);  // Optional: Invert the display (bright text on dark background)

    LOG_DBG("Starting GNSS test application\n");
    gnss_systems_t supported, enabled;
    uint32_t fix_interval;
    int rc;

    rc = gnss_get_supported_systems(GNSS_MODEM, &supported);
    if (rc < 0) {
        LOG_ERR("Failed to query supported systems (%d)\n", rc);
        return rc;
    }
    rc = gnss_get_enabled_systems(GNSS_MODEM, &enabled);
    if (rc < 0) {
        LOG_ERR("Failed to query enabled systems (%d)\n", rc);
        return rc;
    }

    GNSS_SYSTEMS_PRINTF(GNSS_SYSTEM_GPS, supported, enabled);
    GNSS_SYSTEMS_PRINTF(GNSS_SYSTEM_GLONASS, supported, enabled);
    GNSS_SYSTEMS_PRINTF(GNSS_SYSTEM_GALILEO, supported, enabled);
    GNSS_SYSTEMS_PRINTF(GNSS_SYSTEM_BEIDOU, supported, enabled);
    GNSS_SYSTEMS_PRINTF(GNSS_SYSTEM_QZSS, supported, enabled);
    GNSS_SYSTEMS_PRINTF(GNSS_SYSTEM_IRNSS, supported, enabled);
    GNSS_SYSTEMS_PRINTF(GNSS_SYSTEM_SBAS, supported, enabled);
    GNSS_SYSTEMS_PRINTF(GNSS_SYSTEM_IMES, supported, enabled);

    rc = gnss_get_fix_rate(GNSS_MODEM, &fix_interval);
    if (rc < 0) {
        LOG_ERR("Failed to query fix rate (%d)\n", rc);
        return rc;
    }
    LOG_DBG("Fix rate = %d ms\n", fix_interval);

    // Micro-ROS initialization
    printk("Zephyr micro-ROS example starting\n");

    k_sleep(K_MSEC(10)); /* allow rail to stabilize */

    /* Allow system to stabilize */
    k_sleep(K_SECONDS(2));

    /* Configure custom transport */
    rmw_uros_set_custom_transport(true, NULL, zephyr_transport_open, zephyr_transport_close, zephyr_transport_write,
                                  zephyr_transport_read);

    /* Allocator */
    rcl_allocator_t allocator = rcl_get_default_allocator();

    /* Init options with custom domain ID */
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, 10));

    /* micro-ROS support */
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    /* Node */
    node = rcl_get_zero_initialized_node();
    RCCHECK(rclc_node_init_default(&node, "zephyr_publisher", "", &support));

    /* Publisher */
    RCCHECK(rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
                                        "/zephyr_int_publisher"));

    /* Timer */
    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(PUBLISH_PERIOD_MS), timer_callback));

    /* Executor */
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    msg.data = 0;

    /* Start thread */
    k_thread_create(&imu_thread, imu_stack, THREAD_STACK_SIZE, imu_thread_entry, NULL, NULL, NULL, IMU_THREAD_PRIORITY,
                    0, K_NO_WAIT);

    k_thread_name_set(&imu_thread, "imu_thread");

    /* Start executor thread */
    k_thread_create(&executor_thread, executor_stack, EXECUTOR_STACK_SIZE, executor_thread_entry, NULL, NULL, NULL,
                    EXECUTOR_PRIORITY, 0, K_NO_WAIT);

    k_thread_name_set(&executor_thread, "uros_executor");

    /* Start time sync thread */
    k_thread_create(&time_sync_thread, time_sync_stack, TIME_SYNC_STACK_SIZE, time_sync_thread_entry, NULL, NULL, NULL,
                    TIME_SYNC_PRIORITY, 0, K_NO_WAIT);

    k_thread_name_set(&time_sync_thread, "uros_time_sync");

    printk("micro-ROS threads started\n");

    /* main thread does nothing further */
    while (1) {
        k_sleep(K_FOREVER);
    }
}
