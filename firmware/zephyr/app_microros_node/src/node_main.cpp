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

#include <errno.h>
#include <microros_transports.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/string_functions.h>
#include <sensor_msgs/msg/fluid_pressure.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <sensor_msgs/msg/nav_sat_status.h>
#include <sensor_msgs/msg/range.h>
#include <sensor_msgs/msg/temperature.h>
#include <std_msgs/msg/int32.h>
#include <string.h>
#include <time.h>
#include <version.h>
#include <zephyr/device.h>
#include <zephyr/display/cfb.h>
#if DT_NODE_HAS_STATUS(DT_ALIAS(gnss), okay) || DT_NODE_HAS_STATUS(DT_ALIAS(ubloxgnss), okay)
#include <zephyr/drivers/gnss.h>
#endif
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#if DT_NODE_HAS_STATUS(DT_NODELABEL(bno055), okay)
#include <bno055.h>  // Required for custom SENSOR_CHAN_*
#endif
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>

#include "node_callbacks.hpp"
#include "oled_layout.hpp"
#include "oled_wrapper.hpp"
#include "storage.hpp"
#include "wifi_handler.hpp"
static ATOMIC_DEFINE(init_complete, 1);  // single-bit flag, starts 0

LOG_MODULE_REGISTER(all_sensors_module, LOG_LEVEL_DBG);

// Storage instance
Storage storage;

// display driver
static const struct device* display_dev = DEVICE_DT_GET(DT_NODELABEL(ssd1306));
OLEDWrapper oled_wrapper(display_dev);
OLEDLayout oled_layout(&oled_wrapper);
// imu driver
#if DT_NODE_HAS_STATUS(DT_NODELABEL(bno055), okay)
#define BNO055_TIMING_STARTUP 400  // 400ms
static const struct device* const bno055_dev = DEVICE_DT_GET(DT_NODELABEL(bno055));
static bool bno055_fusion = true;
static rcl_publisher_t bno055_imu_publisher;
static rcl_timer_t imu_timer;
static sensor_msgs__msg__Imu bno055_imu_msg;
#endif
// iim42652 driver
#if DT_NODE_HAS_STATUS(DT_NODELABEL(iim42652), okay)
#define IIM42652_TIMING_STARTUP 400  // 400ms
static const struct device* const iim42652_dev = DEVICE_DT_GET(DT_NODELABEL(iim42652));
#else
static const struct device* const iim42652_dev = NULL;
#endif
static rcl_publisher_t iim42652_imu_publisher;
static rcl_timer_t iim42652_timer;
static sensor_msgs__msg__Imu iim42652_imu_msg;
// gnss driver
#if DT_NODE_HAS_STATUS(DT_ALIAS(gnss), okay)
#define mtk3333_gnss DEVICE_DT_GET(DT_ALIAS(gnss))
#endif
#if DT_NODE_HAS_STATUS(DT_ALIAS(ubloxgnss), okay)
#define ublox_gnss DEVICE_DT_GET(DT_ALIAS(ubloxgnss))
#endif
// lps22hb driver
const struct device* st_lps22hb_dev = DEVICE_DT_GET_ANY(st_lps22hb_press);

/* =========================================================
 * Thread Configuration
 * ========================================================= */

#define THREAD_STACK_SIZE 4096

#define EXECUTOR_STACK_SIZE 8192
#define TIME_SYNC_STACK_SIZE 1024

#define EXECUTOR_PRIORITY 4

#define GNSS_PUBLISH_PERIOD_MS 1000
#define IMU_PUBLISH_PERIOD_MS 200
#define IIM42652_PUBLISH_PERIOD_MS 200
#define TIME_SYNC_PERIOD_MS 1000
#define LPS22HB_PUBLISH_PERIOD_MS 2000
#define OLED_VIEW_SWITCH_PERIOD_MS 5000

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
#if DT_NODE_HAS_STATUS(DT_ALIAS(gnss), okay)
static rcl_publisher_t mtk3333_gnss_publisher;
#endif
#if DT_NODE_HAS_STATUS(DT_ALIAS(ubloxgnss), okay)
static rcl_publisher_t ublox_gnss_publisher;
#endif
static rcl_publisher_t lps22hb_temp_publisher;
static rcl_publisher_t lps22hb_pressure_publisher;
// Timers
#if DT_NODE_HAS_STATUS(DT_ALIAS(gnss), okay) || DT_NODE_HAS_STATUS(DT_ALIAS(ubloxgnss), okay)
static rcl_timer_t gnss_timer;
#endif
static rcl_timer_t time_sync_timer;
static rcl_timer_t lps22hb_timer;
static rcl_timer_t oled_view_timer;

static rclc_executor_t executor;
static sensor_msgs__msg__Temperature lps22hb_temp_msg;
static sensor_msgs__msg__FluidPressure lps22hb_pressure_msg;
static std_msgs__msg__Int32 msg;
#if DT_NODE_HAS_STATUS(DT_ALIAS(gnss), okay)
sensor_msgs__msg__NavSatFix mtk3333_nav_sat_fix_msg;
static ATOMIC_DEFINE(mtk3333_msg_ready, 1);
#endif
#if DT_NODE_HAS_STATUS(DT_ALIAS(ubloxgnss), okay)
sensor_msgs__msg__NavSatFix ublox_nav_sat_fix_msg;
static ATOMIC_DEFINE(ublox_msg_ready, 1);
#endif
static atomic_t time_is_valid;
static atomic_t oled_view_switch_request;

#if DT_NODE_HAS_STATUS(DT_ALIAS(sw0), okay)
static const struct gpio_dt_spec view_switch_button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static struct gpio_callback view_switch_button_cb_data;

static void sw1_pressed(const struct device* dev, struct gpio_callback* cb, uint32_t pins) {
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);
    atomic_set(&oled_view_switch_request, 1);
}

static int init_view_switch_button(void) {
    if (!gpio_is_ready_dt(&view_switch_button)) {
        LOG_WRN("View switch button GPIO not ready");
        return -ENODEV;
    }

    int rc = gpio_pin_configure_dt(&view_switch_button, GPIO_INPUT);
    if (rc != 0) {
        LOG_WRN("Failed to configure view switch button (%d)", rc);
        return rc;
    }

    rc = gpio_pin_interrupt_configure_dt(&view_switch_button, GPIO_INT_EDGE_TO_ACTIVE);
    if (rc != 0) {
        LOG_WRN("Failed to enable view switch button interrupt (%d)", rc);
        return rc;
    }

    gpio_init_callback(&view_switch_button_cb_data, sw1_pressed, BIT(view_switch_button.pin));
    rc = gpio_add_callback(view_switch_button.port, &view_switch_button_cb_data);
    if (rc != 0) {
        LOG_WRN("Failed to register view switch callback (%d)", rc);
        return rc;
    }

    LOG_INF("OLED view switch button enabled on pin %d", view_switch_button.pin);
    return 0;
}
#else
static int init_view_switch_button(void) { return -ENOTSUP; }
#endif

/* =========================================================
 * Thread objects
 * ========================================================= */

K_THREAD_STACK_DEFINE(executor_stack, EXECUTOR_STACK_SIZE);
static struct k_thread executor_thread;

/* =========================================================
 * micro-ROS executor thread
 * ========================================================= */

static void executor_thread_entry(void* a, void* b, void* c) {
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        NodeCallbacks::handleExecutorIteration();
        k_sleep(K_MSEC(50));
    }
}

/* =========================================================
 * Time synchronization thread
 * ========================================================= */

#if DT_NODE_HAS_STATUS(DT_ALIAS(gnss), okay)
static void mtk3333_gnss_data_cb(const struct device* dev, const struct gnss_data* data) {
    if (!atomic_test_bit(init_complete, 0)) return;
    uint64_t timepulse_ns;
    k_ticks_t timepulse;
    if (data->info.fix_status != GNSS_FIX_STATUS_NO_FIX) {
        if (gnss_get_latest_timepulse(dev, &timepulse) == 0) {
            timepulse_ns = k_ticks_to_ns_near64(timepulse);
        }
        char buffer[128] = {0};
        LOG_DBG("UTC Time: %02d %02d:%02d", data->utc.month, data->utc.hour, data->utc.minute);
        snprintf(buffer, sizeof(buffer), "%02d:%02d", data->utc.hour, data->utc.minute);

        oled_layout.display_mtk3333_time(buffer);
        oled_layout.finalize_screen();

        rcl_time_point_value_t now = rmw_uros_epoch_nanos();
        mtk3333_nav_sat_fix_msg.header.stamp.sec = now / 1000000000LL;
        mtk3333_nav_sat_fix_msg.header.stamp.nanosec = now % 1000000000LL;

        // ── Position (Zephyr stores as millionths of degrees / mm) ───
        mtk3333_nav_sat_fix_msg.latitude = (double)data->nav_data.latitude / 1e9;
        mtk3333_nav_sat_fix_msg.longitude = (double)data->nav_data.longitude / 1e9;

        mtk3333_nav_sat_fix_msg.altitude = data->nav_data.altitude / 1e3;  // mm → meters
        oled_layout.display_mtk3333_location(mtk3333_nav_sat_fix_msg.latitude, mtk3333_nav_sat_fix_msg.longitude,
                                             mtk3333_nav_sat_fix_msg.altitude);

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
        memset(buffer, 0, sizeof(buffer));
        snprintf(buffer, sizeof(buffer), "MTK3333: Lat=%.6f Lon=%.6f Alt=%.2f Fix=%d", mtk3333_nav_sat_fix_msg.latitude,
                 mtk3333_nav_sat_fix_msg.longitude, mtk3333_nav_sat_fix_msg.altitude, data->info.fix_status);
        storage.log_write(buffer);
        atomic_set_bit(mtk3333_msg_ready, 0);
    }
}
GNSS_DATA_CALLBACK_DEFINE(mtk3333_gnss, mtk3333_gnss_data_cb);

#if CONFIG_GNSS_SATELLITES
static void gnss_satellites_cb(const struct device* dev, const struct gnss_satellite* satellites, uint16_t size) {
    unsigned int tracked_count = 0;
    unsigned int corrected_count = 0;

    for (unsigned int i = 0; i != size; ++i) {
        tracked_count += satellites[i].is_tracked;
        corrected_count += satellites[i].is_corrected;
    }
    oled_layout.display_mtk3333_satellites(tracked_count);
    oled_layout.finalize_screen();
    LOG_DBG("%u satellite%s reported (of which %u tracked, of which %u has RTK corrections)!\n", size,
            size > 1 ? "s" : "", tracked_count, corrected_count);
}
GNSS_SATELLITES_CALLBACK_DEFINE(mtk3333_gnss, gnss_satellites_cb);
#endif /* CONFIG_GNSS_SATELLITES */
#endif /* DT_NODE_HAS_STATUS(DT_ALIAS(gnss), okay) */

#if DT_NODE_HAS_STATUS(DT_ALIAS(ubloxgnss), okay)
static void ublox_gnss_data_cb(const struct device* dev, const struct gnss_data* data) {
    if (!atomic_test_bit(init_complete, 0)) return;
    uint64_t timepulse_ns;
    k_ticks_t timepulse;

    if (data->info.fix_status != GNSS_FIX_STATUS_NO_FIX) {
        if (gnss_get_latest_timepulse(dev, &timepulse) == 0) {
            timepulse_ns = k_ticks_to_ns_near64(timepulse);
        }
    }
    {
        char buffer[128] = {0};
        LOG_DBG("UTC Time: %02d %02d:%02d", data->utc.month, data->utc.hour, data->utc.minute);
        snprintf(buffer, sizeof(buffer), "%02d:%02d", data->utc.hour, data->utc.minute);

        oled_layout.display_ublox_time(buffer);
        oled_layout.finalize_screen();

        rcl_time_point_value_t now = rmw_uros_epoch_nanos();
        ublox_nav_sat_fix_msg.header.stamp.sec = now / 1000000000LL;
        ublox_nav_sat_fix_msg.header.stamp.nanosec = now % 1000000000LL;

        // ── Position (Zephyr stores as millionths of degrees / mm) ───
        ublox_nav_sat_fix_msg.latitude = data->nav_data.latitude / 1e9;  // nanodegrees → degrees
        ublox_nav_sat_fix_msg.longitude = data->nav_data.longitude / 1e9;
        ublox_nav_sat_fix_msg.altitude = data->nav_data.altitude / 1e3;  // mm → meters
        oled_layout.display_ublox_location(ublox_nav_sat_fix_msg.latitude, ublox_nav_sat_fix_msg.longitude,
                                           ublox_nav_sat_fix_msg.altitude);

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
        memset(buffer, 0, sizeof(buffer));
        snprintf(buffer, sizeof(buffer), "Ublox: Lat=%.6f Lon=%.6f Alt=%.2f Fix=%d", ublox_nav_sat_fix_msg.latitude,
                 ublox_nav_sat_fix_msg.longitude, ublox_nav_sat_fix_msg.altitude, data->info.fix_status);
        storage.log_write(buffer);
        atomic_set_bit(ublox_msg_ready, 0);
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
    oled_layout.display_ublox_satellites(tracked_count);
    oled_layout.finalize_screen();
}
GNSS_SATELLITES_CALLBACK_DEFINE(ublox_gnss, ublox_gnss_satellites_cb);
#endif /* CONFIG_GNSS_SATELLITES */
#endif /* DT_NODE_HAS_STATUS(DT_ALIAS(ubloxgnss), okay) */

#if DT_NODE_HAS_STATUS(DT_ALIAS(gnss), okay) || DT_NODE_HAS_STATUS(DT_ALIAS(ubloxgnss), okay)
#define GNSS_SYSTEMS_PRINTF(define, supported, enabled)                                                      \
    LOG_DBG("\t%20s: Supported: %3s Enabled: %3s\n", STRINGIFY(define), (supported & define) ? "Yes" : "No", \
            (enabled & define) ? "Yes" : "No");
#endif

/* =========================================================
 * main()
 * ========================================================= */

int main(void) {
#if DT_NODE_HAS_STATUS(DT_NODELABEL(bno055), okay)
#if Z_DEVICE_DT_FLAGS(DT_NODELABEL(bno055)) & DEVICE_FLAG_INIT_DEFERRED
    LOG_DBG("Deferred init enabled, sleeping for %d ms", BNO055_TIMING_STARTUP);
    k_sleep(K_MSEC(BNO055_TIMING_STARTUP));
    device_init(bno055_dev);
#endif
    sensor_msgs__msg__Imu__init(&bno055_imu_msg);
    rosidl_runtime_c__String__assign(&bno055_imu_msg.header.frame_id, "bno055_imu_frame");
#endif
    sensor_msgs__msg__Temperature__init(&lps22hb_temp_msg);
    rosidl_runtime_c__String__assign(&lps22hb_temp_msg.header.frame_id, "lps22hb_frame");
    lps22hb_temp_msg.variance = 0.0;

    sensor_msgs__msg__FluidPressure__init(&lps22hb_pressure_msg);
    rosidl_runtime_c__String__assign(&lps22hb_pressure_msg.header.frame_id, "lps22hb_frame");
    lps22hb_pressure_msg.variance = 0.0;
#if DT_NODE_HAS_STATUS(DT_ALIAS(gnss), okay)
    sensor_msgs__msg__NavSatFix__init(&mtk3333_nav_sat_fix_msg);
    rosidl_runtime_c__String__assign(&mtk3333_nav_sat_fix_msg.header.frame_id, "mtk3333_gnss_frame");
#endif
#if DT_NODE_HAS_STATUS(DT_ALIAS(ubloxgnss), okay)
    sensor_msgs__msg__NavSatFix__init(&ublox_nav_sat_fix_msg);
    rosidl_runtime_c__String__assign(&ublox_nav_sat_fix_msg.header.frame_id, "ublox_gnss_frame");
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(iim42652), okay)
    sensor_msgs__msg__Imu__init(&iim42652_imu_msg);
    rosidl_runtime_c__String__assign(&iim42652_imu_msg.header.frame_id, "iim42652_imu_frame");
#endif

    if (!device_is_ready(st_lps22hb_dev)) {
        LOG_ERR("LPS22HB device not ready");
        return -ENODEV;
    }

#if DT_NODE_HAS_STATUS(DT_NODELABEL(bno055), okay)
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
#endif

    /* Delay between BNO055 and IIM42652 init to avoid I2C bus contention
     * and allow the first IMU to settle before the second is powered on. */
    k_msleep(100);

#if DT_NODE_HAS_STATUS(DT_NODELABEL(iim42652), okay)
#if Z_DEVICE_DT_FLAGS(DT_NODELABEL(iim42652)) & DEVICE_FLAG_INIT_DEFERRED
    LOG_DBG("Deferred init enabled, sleeping for %d ms", IIM42652_TIMING_STARTUP);
    k_sleep(K_MSEC(IIM42652_TIMING_STARTUP));
    device_init(iim42652_dev);
#endif

    /* Recovery: retry init if chip ID read failed (e.g. I2C bus not settled) */
    for (int _retry = 0; !device_is_ready(iim42652_dev) && _retry < 3; _retry++) {
        LOG_WRN("IIM42652 not ready (attempt %d/3), retrying init in 500 ms...", _retry + 1);
        k_sleep(K_MSEC(500));
        device_init(iim42652_dev);
    }

    if (!device_is_ready(iim42652_dev)) {
        LOG_WRN("IIM42652 failed to initialize after retries, continuing without IMU");
    } else {
        LOG_DBG("IIM42652 Device %s is ready\n", iim42652_dev->name);
    }
#endif
    // Starting display
    if (oled_layout.init_screen() != 0) {
        LOG_ERR("OLED init failed");
        return -ENODEV;
    }

    storage.init();

    storage.print_storage_stats();

    LOG_DBG("Starting GNSS test application\n");
#if DT_NODE_HAS_STATUS(DT_ALIAS(gnss), okay)
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
#endif

    // Micro-ROS initialization
    LOG_DBG("Zephyr micro-ROS example starting\n");

    k_sleep(K_MSEC(10)); /* allow rail to stabilize */

    /* Configure custom transport */
#if defined(CONFIG_MICROROS_TRANSPORT_UDP)
    WifiHandler wifi_handler;
    if (wifi_handler.initStation(oled_layout) != 0) {
        LOG_ERR("WiFi initialization failed");
        char waiting_message[64];
        snprintf(waiting_message, sizeof(waiting_message), "Failed to connect WiFi '%s'", CONFIG_MICROROS_WIFI_SSID);

        oled_layout.display_status_message(waiting_message);
        oled_layout.finalize_screen();
        return -EIO;
    }

    memset(&default_params, 0, sizeof(default_params));
    snprintf(default_params.ip, sizeof(default_params.ip), "%s", CONFIG_MICROROS_AGENT_IP);
    snprintf(default_params.port, sizeof(default_params.port), "%s", CONFIG_MICROROS_AGENT_PORT);

    rmw_uros_set_custom_transport(MICRO_ROS_FRAMING_REQUIRED, &default_params, zephyr_transport_open,
                                  zephyr_transport_close, zephyr_transport_write, zephyr_transport_read);
#else
    rmw_uros_set_custom_transport(true, NULL, zephyr_transport_open, zephyr_transport_close, zephyr_transport_write,
                                  zephyr_transport_read);
#endif

    oled_layout.display_agent_waiting_message();

    LOG_INF("Waiting for micro-ROS agent at %s:%s", CONFIG_MICROROS_AGENT_IP, CONFIG_MICROROS_AGENT_PORT);
    uint32_t agent_wait_round = 0;
    while (rmw_uros_ping_agent(100, 3) != RMW_RET_OK) {
        // Keep ping calls short so WiFi/network threads get enough runtime on constrained targets.
        agent_wait_round++;
        if ((agent_wait_round % 5U) == 0U) {
            LOG_WRN("micro-ROS agent still unreachable (%u checks)", agent_wait_round);
        }
        k_sleep(K_MSEC(500));
    }
    LOG_INF("micro-ROS agent connected");

    oled_layout.show_startup_splash();
    k_sleep(K_MSEC(1800));
    oled_layout.set_display_updates_enabled(true);
    oled_layout.clear_screen();

    oled_layout.set_view(OLEDLayout::View::SATELLITE);
    atomic_clear(&oled_view_switch_request);
    (void)init_view_switch_button();

    /* Allocator */
    rcl_allocator_t allocator = rcl_get_default_allocator();

    /* Init options: use ROS domain ID 10 to match host setup */
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, 10));
    LOG_INF("micro-ROS using ROS domain ID 10");

    /* micro-ROS support */
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    /* Node */
    node = rcl_get_zero_initialized_node();
    RCCHECK(rclc_node_init_default(&node, "sensor_publisher", "", &support));

    /* GNSS publishers reliable for rqt compatibility; IMU best-effort to avoid backpressure stalls. */
#if DT_NODE_HAS_STATUS(DT_ALIAS(gnss), okay)
    RCCHECK(rclc_publisher_init_default(&mtk3333_gnss_publisher, &node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix), "/mtk3333_gnss_raw"));
#endif
#if DT_NODE_HAS_STATUS(DT_ALIAS(ubloxgnss), okay)
    RCCHECK(rclc_publisher_init_default(&ublox_gnss_publisher, &node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix), "/ublox_gnss_raw"));
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(bno055), okay)
    RCCHECK(rclc_publisher_init_default(&bno055_imu_publisher, &node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/bno055_imu_raw"));
#endif
    RCCHECK(rclc_publisher_init_default(&lps22hb_temp_publisher, &node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature),
                                        "/lps22hb_temperature_raw"));
    RCCHECK(rclc_publisher_init_default(&lps22hb_pressure_publisher, &node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, FluidPressure),
                                        "/lps22hb_pressure_raw"));
#if DT_NODE_HAS_STATUS(DT_NODELABEL(iim42652), okay)
    RCCHECK(rclc_publisher_init_default(&iim42652_imu_publisher, &node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/iim42652_imu_raw"));
#endif

    NodeCallbacks::Context callbacks_context = {};
    callbacks_context.oled_layout = &oled_layout;
    callbacks_context.storage = &storage;
    callbacks_context.time_is_valid = &time_is_valid;
    callbacks_context.oled_view_switch_request = &oled_view_switch_request;

#if DT_NODE_HAS_STATUS(DT_NODELABEL(bno055), okay)
    callbacks_context.bno055_dev = bno055_dev;
    callbacks_context.bno055_imu_publisher = &bno055_imu_publisher;
    callbacks_context.bno055_imu_msg = &bno055_imu_msg;
#endif

#if DT_NODE_HAS_STATUS(DT_ALIAS(gnss), okay)
    callbacks_context.mtk3333_gnss_publisher = &mtk3333_gnss_publisher;
    callbacks_context.mtk3333_nav_sat_fix_msg = &mtk3333_nav_sat_fix_msg;
    callbacks_context.mtk3333_msg_ready = mtk3333_msg_ready;
#endif

#if DT_NODE_HAS_STATUS(DT_ALIAS(ubloxgnss), okay)
    callbacks_context.ublox_gnss_publisher = &ublox_gnss_publisher;
    callbacks_context.ublox_nav_sat_fix_msg = &ublox_nav_sat_fix_msg;
    callbacks_context.ublox_msg_ready = ublox_msg_ready;
#endif

    callbacks_context.iim42652_dev = iim42652_dev;
    callbacks_context.iim42652_imu_publisher = &iim42652_imu_publisher;
    callbacks_context.iim42652_imu_msg = &iim42652_imu_msg;
    callbacks_context.st_lps22hb_dev = st_lps22hb_dev;
    callbacks_context.lps22hb_temp_publisher = &lps22hb_temp_publisher;
    callbacks_context.lps22hb_pressure_publisher = &lps22hb_pressure_publisher;
    callbacks_context.lps22hb_temp_msg = &lps22hb_temp_msg;
    callbacks_context.lps22hb_pressure_msg = &lps22hb_pressure_msg;

    NodeCallbacks::initialize(callbacks_context);

    /* Timer */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(bno055), okay)
    RCCHECK(rclc_timer_init_default(&imu_timer, &support, RCL_MS_TO_NS(IMU_PUBLISH_PERIOD_MS),
                                    NodeCallbacks::bno055ImuTimerCallback));
#endif
#if DT_NODE_HAS_STATUS(DT_ALIAS(gnss), okay) || DT_NODE_HAS_STATUS(DT_ALIAS(ubloxgnss), okay)
    RCCHECK(rclc_timer_init_default(&gnss_timer, &support, RCL_MS_TO_NS(GNSS_PUBLISH_PERIOD_MS),
                                    NodeCallbacks::gnssPublishTimerCallback));
#endif
    RCCHECK(rclc_timer_init_default(&time_sync_timer, &support, RCL_MS_TO_NS(TIME_SYNC_PERIOD_MS),
                                    NodeCallbacks::timeSyncTimerCallback));
    RCCHECK(rclc_timer_init_default(&lps22hb_timer, &support, RCL_MS_TO_NS(LPS22HB_PUBLISH_PERIOD_MS),
                                    NodeCallbacks::lps22hbTimerCallback));
    RCCHECK(rclc_timer_init_default(&oled_view_timer, &support, RCL_MS_TO_NS(OLED_VIEW_SWITCH_PERIOD_MS),
                                    NodeCallbacks::oledViewTimerCallback));
#if DT_NODE_HAS_STATUS(DT_NODELABEL(iim42652), okay)
    RCCHECK(rclc_timer_init_default(&iim42652_timer, &support, RCL_MS_TO_NS(IIM42652_PUBLISH_PERIOD_MS),
                                    NodeCallbacks::iim42652TimerCallback));
#endif

    /* Executor */
    RCCHECK(rclc_executor_init(&executor, &support.context, 6, &allocator));

#if DT_NODE_HAS_STATUS(DT_NODELABEL(bno055), okay)
    RCCHECK(rclc_executor_add_timer(&executor, &imu_timer));
#endif
#if DT_NODE_HAS_STATUS(DT_ALIAS(gnss), okay) || DT_NODE_HAS_STATUS(DT_ALIAS(ubloxgnss), okay)
    RCCHECK(rclc_executor_add_timer(&executor, &gnss_timer));
#endif
    RCCHECK(rclc_executor_add_timer(&executor, &time_sync_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &lps22hb_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &oled_view_timer));
#if DT_NODE_HAS_STATUS(DT_NODELABEL(iim42652), okay)
    RCCHECK(rclc_executor_add_timer(&executor, &iim42652_timer));
#endif
    msg.data = 0;

    /* Start executor thread */
    k_thread_create(&executor_thread, executor_stack, EXECUTOR_STACK_SIZE, executor_thread_entry, NULL, NULL, NULL,
                    EXECUTOR_PRIORITY, 0, K_NO_WAIT);

    k_thread_name_set(&executor_thread, "uros_executor");

    LOG_DBG("micro-ROS threads started\n");
    atomic_set_bit(init_complete, 0);  // ← open the gate

    /* main thread does nothing further */
    while (1) {
        k_sleep(K_FOREVER);
    }
}
