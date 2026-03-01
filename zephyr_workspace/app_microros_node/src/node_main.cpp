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
#include <bno055.h>
#include <microros_transports.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/string_functions.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <sensor_msgs/msg/nav_sat_status.h>
#include <sensor_msgs/msg/range.h>
#include <std_msgs/msg/int32.h>
#include <string.h>
#include <version.h>
#include <zephyr/device.h>
#include <zephyr/display/cfb.h>
#include <zephyr/drivers/gnss.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

#include "storage.hpp"

LOG_MODULE_REGISTER(all_sensors_module, LOG_LEVEL_DBG);
#define BNO055_TIMING_STARTUP 400  // 400ms

// Storage instance
Storage storage;

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
#define mtk3333_gnss DEVICE_DT_GET(DT_ALIAS(gnss))
#define ublox_gnss DEVICE_DT_GET(DT_ALIAS(ubloxgnss))

// IMU configuration
static bool bno055_fusion = true;
#define BNO055_TIMING_STARTUP 400

/* =========================================================
 * Thread Configuration
 * ========================================================= */

#define THREAD_STACK_SIZE 4096
#define IMU_THREAD_PRIORITY 6

#define EXECUTOR_STACK_SIZE 4096
#define TIME_SYNC_STACK_SIZE 1024

#define EXECUTOR_PRIORITY 4
#define TIME_SYNC_PRIORITY 5 /* lower priority */

#define GNSS_PUBLISH_PERIOD_MS 1000
#define IMU_PUBLISH_PERIOD_MS 200
#define TIME_SYNC_PERIOD_MS 1000

/* =========================================================
 * Error handling macros
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

/* =========================================================
 * micro-ROS objects
 * ========================================================= */

static rclc_support_t support;
static rcl_node_t node;
// Publishers
static rcl_publisher_t mtk3333_gnss_publisher;
static rcl_publisher_t ublox_gnss_publisher;
static rcl_publisher_t imu_publisher;
// Timers
static rcl_timer_t gnss_timer;
static rcl_timer_t ublox_gnss_timer;
static rcl_timer_t imu_timer;

static rclc_executor_t executor;
static sensor_msgs__msg__Imu imu_msg;
static std_msgs__msg__Int32 msg;
sensor_msgs__msg__NavSatFix mtk3333_nav_sat_fix_msg;
sensor_msgs__msg__NavSatFix ublox_nav_sat_fix_msg;

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

static void gnss_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
    ARG_UNUSED(last_call_time);

    if (timer != NULL) {
        // rcl_time_point_value_t now = rmw_uros_epoch_nanos();
        // mtk3333_nav_sat_fix_msg.header.stamp.sec = now / 1000000000LL;
        // mtk3333_nav_sat_fix_msg.header.stamp.nanosec = now % 1000000000LL;
        // // RCSOFTCHECK(rcl_publish(&mtk3333_gnss_publisher, &mtk3333_nav_sat_fix_msg, NULL));
        // msg.data++;
    }
}

static void ublox_gnss_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
    ARG_UNUSED(last_call_time);

    if (timer != NULL) {
        // rcl_time_point_value_t now = rmw_uros_epoch_nanos();
        // ublox_nav_sat_fix_msg.header.stamp.sec = now / 1000000000LL;
        // ublox_nav_sat_fix_msg.header.stamp.nanosec = now % 1000000000LL;
        // // RCSOFTCHECK(rcl_publish(&ublox_gnss_publisher, &ublox_nav_sat_fix_msg, NULL));
        // msg.data++;
    }
}

static void imu_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
    ARG_UNUSED(last_call_time);
    if (timer != NULL && NULL != bno055_dev) {
        double accel[3] = {0.0, 0.0, 0.0};
        double gyro[3] = {0.0, 0.0, 0.0};
        double mag[3] = {0.0, 0.0, 0.0};
        struct sensor_value eul[3];
        struct sensor_value accel_val[3];
        struct sensor_value gyro_val[3];
        struct sensor_value mag_val[3];
        struct sensor_value quat[4];
        quaternion_data q = {};
        rcl_time_point_value_t now = rmw_uros_epoch_nanos();
        sensor_sample_fetch(bno055_dev);
        sensor_channel_get(bno055_dev, static_cast<sensor_channel>(BNO055_SENSOR_CHAN_EULER_YRP), eul);
        sensor_channel_get(bno055_dev, static_cast<sensor_channel>(BNO055_SENSOR_CHAN_QUATERNION_WXYZ), quat);

        // CORRECT - use integer arithmetic
        imu_msg.header.stamp.sec = now / 1000000000LL;
        imu_msg.header.stamp.nanosec = now % 1000000000LL;

        // Fill accelerometer data
        imu_msg.linear_acceleration.x = accel[0];
        imu_msg.linear_acceleration.y = accel[1];
        imu_msg.linear_acceleration.z = accel[2];

        // Fill gyroscope data
        imu_msg.angular_velocity.x = gyro[0];
        imu_msg.angular_velocity.y = gyro[1];
        imu_msg.angular_velocity.z = gyro[2];

        // Fill orientation
        imu_msg.orientation.w = sensor_value_to_double(&quat[0]);
        imu_msg.orientation.x = sensor_value_to_double(&quat[1]);
        imu_msg.orientation.y = sensor_value_to_double(&quat[2]);
        imu_msg.orientation.z = sensor_value_to_double(&quat[3]);

        // Add covariance if needed
        // For simplicity, leaving covariances as zero
        for (int i = 0; i < 9; ++i) {
            imu_msg.linear_acceleration_covariance[i] = 0.0;
            imu_msg.angular_velocity_covariance[i] = 0.0;
            imu_msg.orientation_covariance[i] = 0.0;
        }

        // Format for the screen
        char buffer[64];
        snprintf(buffer, sizeof(buffer), "X=%.3f", imu_msg.orientation.x);
        cfb_print(display_dev, buffer, 0, 20);

        snprintf(buffer, sizeof(buffer), "Y=%.3f", imu_msg.orientation.y);
        cfb_print(display_dev, buffer, 0, 35);

        snprintf(buffer, sizeof(buffer), "Z=%.3f", imu_msg.orientation.z);
        cfb_print(display_dev, buffer, 0, 50);

        cfb_framebuffer_finalize(display_dev);

        RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
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
        k_sleep(K_MSEC(50));
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
    k_sleep(K_SECONDS(1));
    while (1) {
        bool ok = rmw_uros_sync_session(50); /* 50 ms timeout */

        if (!ok) {
            LOG_DBG("micro-ROS time sync failed\n");
        }
        k_sleep(K_MSEC(TIME_SYNC_PERIOD_MS));
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
        snprintf(buffer, sizeof(buffer), "%02d:%02d", data->utc.hour, data->utc.minute);

        cfb_print(display_dev, buffer, 0, 0);  // Print at
        cfb_framebuffer_finalize(display_dev);

        rcl_time_point_value_t now = rmw_uros_epoch_nanos();
        mtk3333_nav_sat_fix_msg.header.stamp.sec = now / 1000000000LL;
        mtk3333_nav_sat_fix_msg.header.stamp.nanosec = now % 1000000000LL;

        // ── Position (Zephyr stores as millionths of degrees / mm) ───
        mtk3333_nav_sat_fix_msg.latitude = (double)data->nav_data.latitude / 1e9;
        mtk3333_nav_sat_fix_msg.longitude = (double)data->nav_data.longitude / 1e9;

        mtk3333_nav_sat_fix_msg.altitude = data->nav_data.altitude / 1e3;  // mm → meters

        // ── Fix Status ───────────────────────────────────────────────
        switch (data->info.fix_status) {
            case GNSS_FIX_STATUS_GNSS_FIX:
                mtk3333_nav_sat_fix_msg.status.status = sensor_msgs__msg__NavSatStatus__STATUS_FIX;
                break;
            case GNSS_FIX_STATUS_DGNSS_FIX:
                mtk3333_nav_sat_fix_msg.status.status = sensor_msgs__msg__NavSatStatus__STATUS_SBAS_FIX;
                break;
            default:
                mtk3333_nav_sat_fix_msg.status.status = sensor_msgs__msg__NavSatStatus__STATUS_NO_FIX;
                break;
        }

        // ── Service (which constellations) ───────────────────────────
        mtk3333_nav_sat_fix_msg.status.service =
            sensor_msgs__msg__NavSatStatus__SERVICE_GPS | sensor_msgs__msg__NavSatStatus__SERVICE_GLONASS;

        double hdop = data->info.hdop / 1e3;            // if available
        double variance = (hdop * 5.0) * (hdop * 5.0);  // rough estimate

        memset(mtk3333_nav_sat_fix_msg.position_covariance, 0, sizeof(mtk3333_nav_sat_fix_msg.position_covariance));

        mtk3333_nav_sat_fix_msg.position_covariance[0] = variance;        // East
        mtk3333_nav_sat_fix_msg.position_covariance[4] = variance;        // North
        mtk3333_nav_sat_fix_msg.position_covariance[8] = variance * 4.0;  // Up (typically worse)
        mtk3333_nav_sat_fix_msg.position_covariance_type = sensor_msgs__msg__NavSatFix__COVARIANCE_TYPE_APPROXIMATED;

        RCSOFTCHECK(rcl_publish(&mtk3333_gnss_publisher, &mtk3333_nav_sat_fix_msg, NULL));
    }
}
GNSS_DATA_CALLBACK_DEFINE(mtk3333_gnss, gnss_data_cb);

#if CONFIG_GNSS_SATELLITES
static void gnss_satellites_cb(const struct device* dev, const struct gnss_satellite* satellites, uint16_t size) {
    unsigned int tracked_count = 0;
    unsigned int corrected_count = 0;

    for (unsigned int i = 0; i != size; ++i) {
        tracked_count += satellites[i].is_tracked;
        corrected_count += satellites[i].is_corrected;
    }
    LOG_DBG("%u satellite%s reported (of which %u tracked, of which %u has RTK corrections)!\n", size,
            size > 1 ? "s" : "", tracked_count, corrected_count);
}
#endif
GNSS_SATELLITES_CALLBACK_DEFINE(mtk3333_gnss, gnss_satellites_cb);

static void ublox_gnss_data_cb(const struct device* dev, const struct gnss_data* data) {
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
        snprintf(buffer, sizeof(buffer), "%02d:%02d", data->utc.hour, data->utc.minute);

        cfb_print(display_dev, buffer, 64, 0);  // Print at
        cfb_framebuffer_finalize(display_dev);

        rcl_time_point_value_t now = rmw_uros_epoch_nanos();
        ublox_nav_sat_fix_msg.header.stamp.sec = now / 1000000000LL;
        ublox_nav_sat_fix_msg.header.stamp.nanosec = now % 1000000000LL;

        // ── Position (Zephyr stores as millionths of degrees / mm) ───
        ublox_nav_sat_fix_msg.latitude = data->nav_data.latitude / 1e9;  // nanodegrees → degrees
        ublox_nav_sat_fix_msg.longitude = data->nav_data.longitude / 1e9;
        ublox_nav_sat_fix_msg.altitude = data->nav_data.altitude / 1e3;  // mm → meters

        // ── Fix Status ───────────────────────────────────────────────
        switch (data->info.fix_status) {
            case GNSS_FIX_STATUS_GNSS_FIX:
                ublox_nav_sat_fix_msg.status.status = sensor_msgs__msg__NavSatStatus__STATUS_FIX;
                break;
            case GNSS_FIX_STATUS_DGNSS_FIX:
                ublox_nav_sat_fix_msg.status.status = sensor_msgs__msg__NavSatStatus__STATUS_SBAS_FIX;
                break;
            default:
                ublox_nav_sat_fix_msg.status.status = sensor_msgs__msg__NavSatStatus__STATUS_NO_FIX;
                break;
        }

        // ── Service (which constellations) ───────────────────────────
        ublox_nav_sat_fix_msg.status.service =
            sensor_msgs__msg__NavSatStatus__SERVICE_GPS | sensor_msgs__msg__NavSatStatus__SERVICE_GLONASS;

        double hdop = data->info.hdop / 1e3;            // if available
        double variance = (hdop * 5.0) * (hdop * 5.0);  // rough estimate

        memset(ublox_nav_sat_fix_msg.position_covariance, 0, sizeof(ublox_nav_sat_fix_msg.position_covariance));

        ublox_nav_sat_fix_msg.position_covariance[0] = variance;        // East
        ublox_nav_sat_fix_msg.position_covariance[4] = variance;        // North
        ublox_nav_sat_fix_msg.position_covariance[8] = variance * 4.0;  // Up (typically worse)
        ublox_nav_sat_fix_msg.position_covariance_type = sensor_msgs__msg__NavSatFix__COVARIANCE_TYPE_APPROXIMATED;

        RCSOFTCHECK(rcl_publish(&ublox_gnss_publisher, &ublox_nav_sat_fix_msg, NULL));
    }
}
GNSS_DATA_CALLBACK_DEFINE(ublox_gnss, ublox_gnss_data_cb);

#if CONFIG_GNSS_SATELLITES
static void ublox_gnss_satellites_cb(const struct device* dev, const struct gnss_satellite* satellites, uint16_t size) {
    unsigned int tracked_count = 0;
    unsigned int corrected_count = 0;

    for (unsigned int i = 0; i != size; ++i) {
        tracked_count += satellites[i].is_tracked;
        corrected_count += satellites[i].is_corrected;
    }
    LOG_DBG("%u satellite%s reported (of which %u tracked, of which %u has RTK corrections)!\n", size,
            size > 1 ? "s" : "", tracked_count, corrected_count);
}
#endif
GNSS_SATELLITES_CALLBACK_DEFINE(ublox_gnss, ublox_gnss_satellites_cb);

#define GNSS_SYSTEMS_PRINTF(define, supported, enabled)                                                      \
    LOG_DBG("\t%20s: Supported: %3s Enabled: %3s\n", STRINGIFY(define), (supported & define) ? "Yes" : "No", \
            (enabled & define) ? "Yes" : "No");

/* =========================================================
 * main()
 * ========================================================= */

int main(void) {
#if Z_DEVICE_DT_FLAGS(DT_NODELABEL(bno055)) & DEVICE_FLAG_INIT_DEFERRED
    LOG_DBG("Deferred init enabled, sleeping for %d ms", BNO055_TIMING_STARTUP);
    k_sleep(K_MSEC(BNO055_TIMING_STARTUP));
    device_init(bno055_dev);
#endif
    sensor_msgs__msg__Imu__init(&imu_msg);
    rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "bno055_imu_frame");
    sensor_msgs__msg__NavSatFix__init(&mtk3333_nav_sat_fix_msg);
    rosidl_runtime_c__String__assign(&mtk3333_nav_sat_fix_msg.header.frame_id, "mtk3333_gnss_frame");
    sensor_msgs__msg__NavSatFix__init(&ublox_nav_sat_fix_msg);
    rosidl_runtime_c__String__assign(&ublox_nav_sat_fix_msg.header.frame_id, "ublox_gnss_frame");
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

    if (!device_is_ready(bno055_dev)) {
        LOG_ERR("Device %s is not ready\n", bno055_dev->name);
        return -ENODEV;
    }

    struct sensor_value config = {
        .val1 = (bno055_fusion) ? OPERATION_MODE_NDOF : OPERATION_MODE_M4G,
        .val2 = 0,
    };
    sensor_attr_set(bno055_dev, SENSOR_CHAN_ALL, SENSOR_ATTR_CONFIGURATION, &config);
    config.val1 = BNO055_POWER_NORMAL;
    config.val2 = 0;
    sensor_attr_set(bno055_dev, SENSOR_CHAN_ALL, static_cast<sensor_attribute>(BNO055_SENSOR_ATTR_POWER_MODE), &config);

    cfb_framebuffer_clear(display_dev, true);

    display_blanking_off(display_dev);

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

    storage.init();

    LOG_DBG("Starting GNSS test application\n");
    gnss_systems_t supported, enabled;
    uint32_t fix_interval;
    int rc;

    rc = gnss_get_supported_systems(mtk3333_gnss, &supported);
    if (rc < 0) {
        LOG_ERR("Failed to query supported systems (%d)\n", rc);
        return rc;
    }
    rc = gnss_get_enabled_systems(mtk3333_gnss, &enabled);
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

    rc = gnss_get_fix_rate(mtk3333_gnss, &fix_interval);
    if (rc < 0) {
        LOG_ERR("Failed to query fix rate (%d)\n", rc);
        return rc;
    }
    LOG_DBG("Fix rate = %d ms\n", fix_interval);

    // Micro-ROS initialization
    LOG_DBG("Zephyr micro-ROS example starting\n");

    k_sleep(K_MSEC(10)); /* allow rail to stabilize */

    /* Configure custom transport */
    rmw_uros_set_custom_transport(true, NULL, zephyr_transport_open, zephyr_transport_close, zephyr_transport_write,
                                  zephyr_transport_read);

    char waiting_message[64];
    snprintf(waiting_message, sizeof(waiting_message), "Waiting for ROS Agent");

    cfb_print(display_dev, waiting_message, 0, 0);  // Print at
    cfb_framebuffer_finalize(display_dev);

    LOG_DBG("Waiting for micro-ROS agent...\n");
    while (rmw_uros_ping_agent(100, 10) != RMW_RET_OK) {
        // 100ms timeout, 10 attempts per call
        LOG_DBG("Agent not reachable, retrying...\n");
        k_sleep(K_MSEC(1000));
    }
    LOG_DBG("Agent connected!\n");
    cfb_framebuffer_clear(display_dev, true);

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
    RCCHECK(rclc_node_init_default(&node, "sensor_publisher", "", &support));

    /* Publisher */
    RCCHECK(rclc_publisher_init_default(&mtk3333_gnss_publisher, &node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix), "/gnss_raw"));
    RCCHECK(rclc_publisher_init_default(&ublox_gnss_publisher, &node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix), "/ublox_gnss_raw"));

    RCCHECK(rclc_publisher_init_default(&imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
                                        "/imu_raw"));

    /* Timer */
    RCCHECK(rclc_timer_init_default(&gnss_timer, &support, RCL_MS_TO_NS(GNSS_PUBLISH_PERIOD_MS), gnss_timer_callback));
    RCCHECK(rclc_timer_init_default(&ublox_gnss_timer, &support, RCL_MS_TO_NS(GNSS_PUBLISH_PERIOD_MS),
                                    ublox_gnss_timer_callback));
    RCCHECK(rclc_timer_init_default(&imu_timer, &support, RCL_MS_TO_NS(IMU_PUBLISH_PERIOD_MS), imu_timer_callback));

    /* Executor */
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));

    RCCHECK(rclc_executor_add_timer(&executor, &gnss_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &ublox_gnss_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &imu_timer));

    msg.data = 0;

    /* Start executor thread */
    k_thread_create(&executor_thread, executor_stack, EXECUTOR_STACK_SIZE, executor_thread_entry, NULL, NULL, NULL,
                    EXECUTOR_PRIORITY, 0, K_NO_WAIT);

    k_thread_name_set(&executor_thread, "uros_executor");

    /* Start time sync thread */
    k_thread_create(&time_sync_thread, time_sync_stack, TIME_SYNC_STACK_SIZE, time_sync_thread_entry, NULL, NULL, NULL,
                    TIME_SYNC_PRIORITY, 0, K_NO_WAIT);

    k_thread_name_set(&time_sync_thread, "uros_time_sync");

    LOG_DBG("micro-ROS threads started\n");

    /* main thread does nothing further */
    while (1) {
        k_sleep(K_FOREVER);
    }
}
