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
#include <zephyr/drivers/gnss.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#if defined(CONFIG_MICROROS_TRANSPORT_UDP)
#include <zephyr/net/net_event.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/wifi_mgmt.h>
#endif
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>

#include "oled_wrapper.hpp"
#include "storage.hpp"
static ATOMIC_DEFINE(init_complete, 1);  // single-bit flag, starts 0

LOG_MODULE_REGISTER(all_sensors_module, LOG_LEVEL_DBG);
#define BNO055_TIMING_STARTUP 400  // 400ms

// Storage instance
Storage storage;

// display driver
static const struct device* display_dev = DEVICE_DT_GET(DT_NODELABEL(ssd1306));
OLEDWrapper oled(display_dev);
// imu driver
static const struct device* const bno055_dev = DEVICE_DT_GET(DT_NODELABEL(bno055));
// gnss driver
#define mtk3333_gnss DEVICE_DT_GET(DT_ALIAS(gnss))
#define ublox_gnss DEVICE_DT_GET(DT_ALIAS(ubloxgnss))
// lps22hb driver
const struct device* st_lps22hb_dev = DEVICE_DT_GET_ANY(st_lps22hb_press);

// IMU configuration
static bool bno055_fusion = true;
#define BNO055_TIMING_STARTUP 400

/* =========================================================
 * Thread Configuration
 * ========================================================= */

#define THREAD_STACK_SIZE 4096

#define EXECUTOR_STACK_SIZE 8192
#define TIME_SYNC_STACK_SIZE 1024

#define EXECUTOR_PRIORITY 4

#define GNSS_PUBLISH_PERIOD_MS 1000
#define IMU_PUBLISH_PERIOD_MS 200
#define TIME_SYNC_PERIOD_MS 1000
#define LPS22HB_PUBLISH_PERIOD_MS 2000

#if defined(CONFIG_MICROROS_TRANSPORT_UDP)
#define WIFI_CONNECT_TIMEOUT_S 30
#define WIFI_MAX_RETRIES 5
#define WIFI_RETRY_DELAY_S 5
#endif

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
static rcl_publisher_t bno055_imu_publisher;
static rcl_publisher_t lps22hb_temp_publisher;
static rcl_publisher_t lps22hb_pressure_publisher;
// Timers
static rcl_timer_t imu_timer;
static rcl_timer_t gnss_timer;
static rcl_timer_t time_sync_timer;
static rcl_timer_t lps22hb_timer;

static rclc_executor_t executor;
static sensor_msgs__msg__Imu bno055_imu_msg;
static sensor_msgs__msg__Temperature lps22hb_temp_msg;
static sensor_msgs__msg__FluidPressure lps22hb_pressure_msg;
static std_msgs__msg__Int32 msg;
sensor_msgs__msg__NavSatFix mtk3333_nav_sat_fix_msg;
sensor_msgs__msg__NavSatFix ublox_nav_sat_fix_msg;
static ATOMIC_DEFINE(mtk3333_msg_ready, 1);
static ATOMIC_DEFINE(ublox_msg_ready, 1);
static atomic_t time_is_valid;

#if defined(CONFIG_MICROROS_TRANSPORT_UDP)
static K_SEM_DEFINE(wifi_connected_sem, 0, 1);
static K_SEM_DEFINE(wifi_disconnected_sem, 0, 1);

static struct net_mgmt_event_callback wifi_cb;
static struct net_mgmt_event_callback ipv4_cb;
static struct net_if* sta_iface;

static void wifi_event_handler(struct net_mgmt_event_callback* cb, uint64_t mgmt_event, struct net_if* iface) {
    ARG_UNUSED(iface);

    if (mgmt_event == NET_EVENT_WIFI_CONNECT_RESULT) {
        const struct wifi_status* status = static_cast<const struct wifi_status*>(cb->info);
        if (status && status->status == 0) {
            LOG_INF("WiFi connected");
            k_sem_give(&wifi_connected_sem);
        } else {
            LOG_ERR("WiFi connect failed (status=%d)", status ? status->status : -1);
            k_sem_give(&wifi_disconnected_sem);
        }
    } else if (mgmt_event == NET_EVENT_WIFI_DISCONNECT_RESULT) {
        LOG_WRN("WiFi disconnected");
        k_sem_give(&wifi_disconnected_sem);
    }
}

static void ipv4_event_handler(struct net_mgmt_event_callback* cb, uint64_t mgmt_event, struct net_if* iface) {
    ARG_UNUSED(cb);

    if (mgmt_event != NET_EVENT_IPV4_ADDR_ADD) {
        return;
    }

    struct net_if_ipv4* ipv4_cfg = NULL;
    if (net_if_config_ipv4_get(iface, &ipv4_cfg) < 0 || !ipv4_cfg) {
        return;
    }

    char ip_buf[NET_IPV4_ADDR_LEN];
    for (int i = 0; i < NET_IF_MAX_IPV4_ADDR; ++i) {
        struct net_if_addr_ipv4* entry = &ipv4_cfg->unicast[i];
        if (entry->ipv4.is_added) {
            LOG_INF("DHCP IP: %s", net_addr_ntop(AF_INET, &entry->ipv4.address.in_addr, ip_buf, sizeof(ip_buf)));
            return;
        }
    }
}

static int wifi_try_connect(void) {
    k_sem_reset(&wifi_connected_sem);
    k_sem_reset(&wifi_disconnected_sem);

    struct wifi_connect_req_params params;
    memset(&params, 0, sizeof(params));
    params.ssid = reinterpret_cast<const uint8_t*>(CONFIG_MICROROS_WIFI_SSID);
    params.ssid_length = sizeof(CONFIG_MICROROS_WIFI_SSID) - 1;
    params.psk = reinterpret_cast<const uint8_t*>(CONFIG_MICROROS_WIFI_PASSWORD);
    params.psk_length = sizeof(CONFIG_MICROROS_WIFI_PASSWORD) - 1;
    params.sae_password = NULL;
    params.sae_password_length = 0;
    params.channel = WIFI_CHANNEL_ANY;
    params.band = WIFI_FREQ_BAND_2_4_GHZ;
    params.security = (params.psk_length > 0) ? WIFI_SECURITY_TYPE_PSK : WIFI_SECURITY_TYPE_NONE;
    params.mfp = WIFI_MFP_OPTIONAL;

    int ret = net_mgmt(NET_REQUEST_WIFI_CONNECT, sta_iface, &params, sizeof(params));
    if (ret) {
        LOG_ERR("Connect request rejected (%d)", ret);
        return ret;
    }

    if (k_sem_take(&wifi_connected_sem, K_SECONDS(WIFI_CONNECT_TIMEOUT_S)) == 0) {
        return 0;
    }

    LOG_WRN("WiFi connect timeout");
    return -ETIMEDOUT;
}

static int wifi_connect_with_retry(void) {
    if (!sta_iface) {
        return -ENODEV;
    }

    for (int attempt = 1; attempt <= WIFI_MAX_RETRIES; ++attempt) {
        LOG_INF("Connecting to WiFi SSID '%s' (%d/%d)", CONFIG_MICROROS_WIFI_SSID, attempt, WIFI_MAX_RETRIES);
        if (wifi_try_connect() == 0) {
            return 0;
        }

        if (attempt < WIFI_MAX_RETRIES) {
            k_sleep(K_SECONDS(WIFI_RETRY_DELAY_S));
        }
    }

    return -ETIMEDOUT;
}

static int init_wifi_station(void) {
    net_mgmt_init_event_callback(&wifi_cb, wifi_event_handler,
                                 NET_EVENT_WIFI_CONNECT_RESULT | NET_EVENT_WIFI_DISCONNECT_RESULT);
    net_mgmt_add_event_callback(&wifi_cb);

    net_mgmt_init_event_callback(&ipv4_cb, ipv4_event_handler, NET_EVENT_IPV4_ADDR_ADD);
    net_mgmt_add_event_callback(&ipv4_cb);
    char waiting_message[64];
    snprintf(waiting_message, sizeof(waiting_message), "Waiting for Wifi '%s'...", CONFIG_MICROROS_WIFI_SSID);

    oled.print(waiting_message, 0, 0);  // Print at
    oled.finalize();

    k_sleep(K_SECONDS(2));

    sta_iface = net_if_get_wifi_sta();
    if (!sta_iface) {
        LOG_ERR("No STA interface available. Enable WiFi board support.");
        return -ENODEV;
    }

    return wifi_connect_with_retry();
}
#endif

/* =========================================================
 * Thread objects
 * ========================================================= */

K_THREAD_STACK_DEFINE(executor_stack, EXECUTOR_STACK_SIZE);
static struct k_thread executor_thread;

/* =========================================================
 * Timer callback (runs inside executor thread)
 * ========================================================= */

static void bno055_imu_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
    ARG_UNUSED(last_call_time);
    if (timer != NULL && NULL != bno055_dev) {
        double accel[3] = {0.0, 0.0, 0.0};
        double gyro[3] = {0.0, 0.0, 0.0};
        struct sensor_value eul[3];
        struct sensor_value quat[4];
        rcl_time_point_value_t now = rmw_uros_epoch_nanos();
        sensor_sample_fetch(bno055_dev);
        sensor_channel_get(bno055_dev, static_cast<sensor_channel>(BNO055_SENSOR_CHAN_EULER_YRP), eul);
        sensor_channel_get(bno055_dev, static_cast<sensor_channel>(BNO055_SENSOR_CHAN_QUATERNION_WXYZ), quat);

        // CORRECT - use integer arithmetic
        bno055_imu_msg.header.stamp.sec = now / 1000000000LL;
        bno055_imu_msg.header.stamp.nanosec = now % 1000000000LL;

        // Fill accelerometer data
        bno055_imu_msg.linear_acceleration.x = accel[0];
        bno055_imu_msg.linear_acceleration.y = accel[1];
        bno055_imu_msg.linear_acceleration.z = accel[2];

        // Fill gyroscope data
        bno055_imu_msg.angular_velocity.x = gyro[0];
        bno055_imu_msg.angular_velocity.y = gyro[1];
        bno055_imu_msg.angular_velocity.z = gyro[2];

        // Fill orientation
        bno055_imu_msg.orientation.w = sensor_value_to_double(&quat[0]);
        bno055_imu_msg.orientation.x = sensor_value_to_double(&quat[1]);
        bno055_imu_msg.orientation.y = sensor_value_to_double(&quat[2]);
        bno055_imu_msg.orientation.z = sensor_value_to_double(&quat[3]);

        // Add covariance if needed
        // For simplicity, leaving covariances as zero
        for (int i = 0; i < 9; ++i) {
            bno055_imu_msg.linear_acceleration_covariance[i] = 0.0;
            bno055_imu_msg.angular_velocity_covariance[i] = 0.0;
            bno055_imu_msg.orientation_covariance[i] = 0.0;
        }

        LOG_DBG("Publishing IMU quat x=%.3f y=%.3f z=%.3f", bno055_imu_msg.orientation.x, bno055_imu_msg.orientation.y,
                bno055_imu_msg.orientation.z);
        // Format for the screen
        char buffer[64];
        snprintf(buffer, sizeof(buffer), "X=%.1f", bno055_imu_msg.orientation.x);
        oled.print(buffer, 0, 20);

        snprintf(buffer, sizeof(buffer), "Y=%.1f", bno055_imu_msg.orientation.y);
        oled.print(buffer, 0, 35);

        snprintf(buffer, sizeof(buffer), "Z=%.1f", bno055_imu_msg.orientation.z);
        oled.print(buffer, 0, 50);

        oled.finalize();
        memset(buffer, 0, sizeof(buffer));
        snprintf(buffer, sizeof(buffer), "X=%.3f Y=%.3f Z=%.3f", bno055_imu_msg.orientation.x,
                 bno055_imu_msg.orientation.y, bno055_imu_msg.orientation.z);
        storage.log_write(buffer);

        RCSOFTCHECK(rcl_publish(&bno055_imu_publisher, &bno055_imu_msg, NULL));
    }
}

static void gnss_publish_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
    ARG_UNUSED(last_call_time);

    if (timer == NULL) {
        return;
    }

    if (atomic_test_and_clear_bit(mtk3333_msg_ready, 0)) {
        RCSOFTCHECK(rcl_publish(&mtk3333_gnss_publisher, &mtk3333_nav_sat_fix_msg, NULL));
    }

    if (atomic_test_and_clear_bit(ublox_msg_ready, 0)) {
        RCSOFTCHECK(rcl_publish(&ublox_gnss_publisher, &ublox_nav_sat_fix_msg, NULL));
    }
}

static void time_sync_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
    ARG_UNUSED(last_call_time);

    if (timer == NULL) {
        return;
    }

    if (rmw_uros_sync_session(50) != RMW_RET_OK) {
        LOG_DBG("micro-ROS time sync failed");
        return;
    }

    int64_t epoch_ms = rmw_uros_epoch_millis();
    if (epoch_ms <= 0) {
        LOG_WRN("Invalid epoch from agent, skipping clock update");
        return;
    }

    struct timespec ts = {
        .tv_sec = (time_t)(epoch_ms / 1000),
        .tv_nsec = (long)((epoch_ms % 1000) * 1000000L),
    };

    if (clock_settime(CLOCK_REALTIME, &ts) == 0) {
        atomic_set(&time_is_valid, 1);
    }
}

static void lps22hb_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
    ARG_UNUSED(last_call_time);
    if (timer != NULL) {
        struct sensor_value temp, press;
        int rc = sensor_sample_fetch(st_lps22hb_dev);
        if (rc < 0) {
            LOG_ERR("LPS22HB sensor_sample_fetch error: %d", rc);
            return;
        }

        rc = sensor_channel_get(st_lps22hb_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
        if (rc < 0) {
            LOG_ERR("LPS22HB sensor_channel_get ambient temp error: %d", rc);
            return;
        }

        rc = sensor_channel_get(st_lps22hb_dev, SENSOR_CHAN_PRESS, &press);
        if (rc < 0) {
            LOG_ERR("LPS22HB sensor_channel_get pressure error: %d", rc);
            return;
        }

        rcl_time_point_value_t now = rmw_uros_epoch_nanos();
        lps22hb_temp_msg.header.stamp.sec = now / 1000000000LL;
        lps22hb_temp_msg.header.stamp.nanosec = now % 1000000000LL;
        lps22hb_pressure_msg.header.stamp = lps22hb_temp_msg.header.stamp;

        lps22hb_temp_msg.temperature = sensor_value_to_double(&temp);
        lps22hb_pressure_msg.fluid_pressure = sensor_value_to_double(&press) * 1000.0;

        RCSOFTCHECK(rcl_publish(&lps22hb_temp_publisher, &lps22hb_temp_msg, NULL));
        RCSOFTCHECK(rcl_publish(&lps22hb_pressure_publisher, &lps22hb_pressure_msg, NULL));

        char buffer[64];
        snprintf(buffer, sizeof(buffer), "T&P");
        oled.print(buffer, 70, 20);

        snprintf(buffer, sizeof(buffer), "%d.%01dC", temp.val1, abs(temp.val2) / 10000);
        oled.print(buffer, 70, 35);

        snprintf(buffer, sizeof(buffer), "%d.%01dkPa", press.val1, abs(press.val2) / 10000);
        oled.print(buffer, 70, 50);

        // storage log
        memset(buffer, 0, sizeof(buffer));
        snprintf(buffer, sizeof(buffer), "T=%d.%02dC P=%d.%02dkPa", temp.val1, abs(temp.val2) / 10000, press.val1,
                 abs(press.val2) / 10000);
        storage.log_write(buffer);

        oled.finalize();
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

        oled.print(buffer, 64, 0);  // Print at
        oled.finalize();

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
    LOG_DBG("%u satellite%s reported (of which %u tracked, of which %u has RTK corrections)!\n", size,
            size > 1 ? "s" : "", tracked_count, corrected_count);
}
#endif
GNSS_SATELLITES_CALLBACK_DEFINE(mtk3333_gnss, gnss_satellites_cb);

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

        oled.print(buffer, 0, 0);  // Print at
        oled.finalize();

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
    sensor_msgs__msg__Imu__init(&bno055_imu_msg);
    rosidl_runtime_c__String__assign(&bno055_imu_msg.header.frame_id, "bno055_imu_frame");
    sensor_msgs__msg__Temperature__init(&lps22hb_temp_msg);
    rosidl_runtime_c__String__assign(&lps22hb_temp_msg.header.frame_id, "lps22hb_frame");
    lps22hb_temp_msg.variance = 0.0;

    sensor_msgs__msg__FluidPressure__init(&lps22hb_pressure_msg);
    rosidl_runtime_c__String__assign(&lps22hb_pressure_msg.header.frame_id, "lps22hb_frame");
    lps22hb_pressure_msg.variance = 0.0;

    sensor_msgs__msg__NavSatFix__init(&mtk3333_nav_sat_fix_msg);
    rosidl_runtime_c__String__assign(&mtk3333_nav_sat_fix_msg.header.frame_id, "mtk3333_gnss_frame");
    sensor_msgs__msg__NavSatFix__init(&ublox_nav_sat_fix_msg);
    rosidl_runtime_c__String__assign(&ublox_nav_sat_fix_msg.header.frame_id, "ublox_gnss_frame");

    if (!device_is_ready(st_lps22hb_dev)) {
        LOG_ERR("LPS22HB device not ready");
        return -ENODEV;
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
    // Starting display
    oled.init();
    storage.init();

    storage.print_storage_stats();

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
#if defined(CONFIG_MICROROS_TRANSPORT_UDP)
    if (init_wifi_station() != 0) {
        LOG_ERR("WiFi initialization failed");
        char waiting_message[64];
        snprintf(waiting_message, sizeof(waiting_message), "Failed to connect WiFi '%s'", CONFIG_MICROROS_WIFI_SSID);

        oled.print(waiting_message, 0, 0);  // Print at
        oled.finalize();
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

    char waiting_message[64];
    snprintf(waiting_message, sizeof(waiting_message), "Waiting for ROS Agent");

    oled.print(waiting_message, 0, 0);  // Print at
    oled.finalize();

    LOG_DBG("Waiting for micro-ROS agent...\n");
    while (rmw_uros_ping_agent(100, 10) != RMW_RET_OK) {
        // 100ms timeout, 10 attempts per call
        LOG_DBG("Agent not reachable, retrying...\n");
        k_sleep(K_MSEC(1000));
    }
    LOG_DBG("Agent connected!\n");
    oled.clear();

    oled.draw_horizontal_line(18, 0, 128);
    oled.draw_vertical_line(68, 0, 64);

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
    RCCHECK(rclc_publisher_init_default(&mtk3333_gnss_publisher, &node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix), "/mtk3333_gnss_raw"));
    RCCHECK(rclc_publisher_init_default(&ublox_gnss_publisher, &node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix), "/ublox_gnss_raw"));

    RCCHECK(rclc_publisher_init_default(&bno055_imu_publisher, &node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/bno055_imu_raw"));
    RCCHECK(rclc_publisher_init_default(&lps22hb_temp_publisher, &node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature),
                                        "/lps22hb_temperature_raw"));
    RCCHECK(rclc_publisher_init_default(&lps22hb_pressure_publisher, &node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, FluidPressure),
                                        "/lps22hb_pressure_raw"));

    /* Timer */
    RCCHECK(
        rclc_timer_init_default(&imu_timer, &support, RCL_MS_TO_NS(IMU_PUBLISH_PERIOD_MS), bno055_imu_timer_callback));
    RCCHECK(rclc_timer_init_default(&gnss_timer, &support, RCL_MS_TO_NS(GNSS_PUBLISH_PERIOD_MS),
                                    gnss_publish_timer_callback));
    RCCHECK(rclc_timer_init_default(&time_sync_timer, &support, RCL_MS_TO_NS(TIME_SYNC_PERIOD_MS),
                                    time_sync_timer_callback));
    RCCHECK(rclc_timer_init_default(&lps22hb_timer, &support, RCL_MS_TO_NS(LPS22HB_PUBLISH_PERIOD_MS),
                                    lps22hb_timer_callback));

    /* Executor */
    RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));

    RCCHECK(rclc_executor_add_timer(&executor, &imu_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &gnss_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &time_sync_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &lps22hb_timer));
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
