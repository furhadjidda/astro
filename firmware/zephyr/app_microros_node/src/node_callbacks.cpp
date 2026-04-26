#include "node_callbacks.hpp"

#include <rmw_microros/rmw_microros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <zephyr/drivers/sensor.h>
#if DT_NODE_HAS_STATUS(DT_NODELABEL(bno055), okay)
#include <bno055.h>
#endif
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "oled_layout.hpp"
#include "storage.hpp"

#define RCSOFTCHECK(fn)                                           \
    do {                                                          \
        rcl_ret_t rc = (fn);                                      \
        if (rc != RCL_RET_OK) {                                   \
            LOG_DBG("RCL warning %d at line %d\n", rc, __LINE__); \
        }                                                         \
    } while (0)

LOG_MODULE_DECLARE(all_sensors_module, LOG_LEVEL_DBG);

NodeCallbacks::Context NodeCallbacks::ctx_ = {};
bool NodeCallbacks::initialized_ = false;

void NodeCallbacks::initialize(const Context& context) {
    ctx_ = context;
    initialized_ = true;
}

void NodeCallbacks::handleExecutorIteration() {
    if (!initialized_ || ctx_.oled_layout == nullptr || ctx_.oled_view_switch_request == nullptr) {
        return;
    }

    if (atomic_cas(ctx_.oled_view_switch_request, 1, 0)) {
        LOG_INF("OLED button view switch");
        ctx_.oled_layout->next_view();
    }
}

#if DT_NODE_HAS_STATUS(DT_NODELABEL(bno055), okay)
void NodeCallbacks::bno055ImuTimerCallback(rcl_timer_t* timer, int64_t last_call_time) {
    ARG_UNUSED(last_call_time);
    if (!initialized_ || timer == NULL || ctx_.bno055_dev == NULL || ctx_.bno055_imu_msg == nullptr ||
        ctx_.bno055_imu_publisher == nullptr || ctx_.oled_layout == nullptr || ctx_.storage == nullptr) {
        return;
    }

    struct sensor_value eul[3];
    struct sensor_value quat[4];
    struct sensor_value accel[4];
    struct sensor_value gyro[4];

    int rc = sensor_sample_fetch(ctx_.bno055_dev);
    if (rc < 0) {
        LOG_ERR("BNO055 sensor_sample_fetch error: %d", rc);
        return;
    }

    rc = sensor_channel_get(ctx_.bno055_dev, static_cast<sensor_channel>(BNO055_SENSOR_CHAN_EULER_YRP), eul);
    if (rc < 0) {
        LOG_ERR("BNO055 sensor_channel_get euler error: %d", rc);
        return;
    }

    rc = sensor_channel_get(ctx_.bno055_dev, static_cast<sensor_channel>(BNO055_SENSOR_CHAN_QUATERNION_WXYZ), quat);
    if (rc < 0) {
        LOG_ERR("BNO055 sensor_channel_get quaternion error: %d", rc);
        return;
    }

    rc = sensor_channel_get(ctx_.bno055_dev, static_cast<sensor_channel>(BNO055_SENSOR_CHAN_LINEAR_ACCEL_XYZ), accel);
    if (rc < 0) {
        LOG_ERR("BNO055 sensor_channel_get accel error: %d", rc);
        return;
    }

    rc = sensor_channel_get(ctx_.bno055_dev, static_cast<sensor_channel>(BNO055_SENSOR_CHAN_GRAVITY_XYZ), gyro);
    if (rc < 0) {
        LOG_ERR("BNO055 sensor_channel_get gyro error: %d", rc);
        return;
    }

    rcl_time_point_value_t now = rmw_uros_epoch_nanos();

    ctx_.bno055_imu_msg->header.stamp.sec = now / 1000000000LL;
    ctx_.bno055_imu_msg->header.stamp.nanosec = now % 1000000000LL;

    ctx_.bno055_imu_msg->linear_acceleration.x = sensor_value_to_double(&accel[0]);
    ctx_.bno055_imu_msg->linear_acceleration.y = sensor_value_to_double(&accel[1]);
    ctx_.bno055_imu_msg->linear_acceleration.z = sensor_value_to_double(&accel[2]);

    ctx_.bno055_imu_msg->angular_velocity.x = sensor_value_to_double(&gyro[0]);
    ctx_.bno055_imu_msg->angular_velocity.y = sensor_value_to_double(&gyro[1]);
    ctx_.bno055_imu_msg->angular_velocity.z = sensor_value_to_double(&gyro[2]);

    ctx_.bno055_imu_msg->orientation.w = sensor_value_to_double(&quat[0]);
    ctx_.bno055_imu_msg->orientation.x = sensor_value_to_double(&quat[1]);
    ctx_.bno055_imu_msg->orientation.y = sensor_value_to_double(&quat[2]);
    ctx_.bno055_imu_msg->orientation.z = sensor_value_to_double(&quat[3]);

    for (int i = 0; i < 9; ++i) {
        ctx_.bno055_imu_msg->linear_acceleration_covariance[i] = 0.0;
        ctx_.bno055_imu_msg->angular_velocity_covariance[i] = 0.0;
        ctx_.bno055_imu_msg->orientation_covariance[i] = 0.0;
    }

    LOG_DBG("Publishing IMU quat x=%.3f y=%.3f z=%.3f", ctx_.bno055_imu_msg->orientation.x,
            ctx_.bno055_imu_msg->orientation.y, ctx_.bno055_imu_msg->orientation.z);

    char buffer[64];
    ctx_.oled_layout->display_imu_orientation(ctx_.bno055_imu_msg->orientation.x, ctx_.bno055_imu_msg->orientation.y,
                                              ctx_.bno055_imu_msg->orientation.z);
    ctx_.oled_layout->finalize_screen();
    memset(buffer, 0, sizeof(buffer));
    snprintf(buffer, sizeof(buffer), "X=%.3f Y=%.3f Z=%.3f", ctx_.bno055_imu_msg->orientation.x,
             ctx_.bno055_imu_msg->orientation.y, ctx_.bno055_imu_msg->orientation.z);
    ctx_.storage->log_write(buffer);

    RCSOFTCHECK(rcl_publish(ctx_.bno055_imu_publisher, ctx_.bno055_imu_msg, NULL));
}
#endif

#if DT_NODE_HAS_STATUS(DT_ALIAS(gnss), okay) || DT_NODE_HAS_STATUS(DT_ALIAS(ubloxgnss), okay)
void NodeCallbacks::gnssPublishTimerCallback(rcl_timer_t* timer, int64_t last_call_time) {
    ARG_UNUSED(last_call_time);

    if (!initialized_ || timer == NULL) {
        return;
    }

#if DT_NODE_HAS_STATUS(DT_ALIAS(gnss), okay)
    if (ctx_.mtk3333_msg_ready != nullptr && ctx_.mtk3333_gnss_publisher != nullptr &&
        ctx_.mtk3333_nav_sat_fix_msg != nullptr && atomic_test_and_clear_bit(ctx_.mtk3333_msg_ready, 0)) {
        RCSOFTCHECK(rcl_publish(ctx_.mtk3333_gnss_publisher, ctx_.mtk3333_nav_sat_fix_msg, NULL));
    }
#endif

#if DT_NODE_HAS_STATUS(DT_ALIAS(ubloxgnss), okay)
    if (ctx_.ublox_msg_ready != nullptr && ctx_.ublox_gnss_publisher != nullptr &&
        ctx_.ublox_nav_sat_fix_msg != nullptr && atomic_test_and_clear_bit(ctx_.ublox_msg_ready, 0)) {
        RCSOFTCHECK(rcl_publish(ctx_.ublox_gnss_publisher, ctx_.ublox_nav_sat_fix_msg, NULL));
    }
#endif
}
#endif

void NodeCallbacks::oledViewTimerCallback(rcl_timer_t* timer, int64_t last_call_time) {
    ARG_UNUSED(last_call_time);

    if (!initialized_ || timer == NULL || ctx_.oled_layout == nullptr) {
        return;
    }

    LOG_INF("OLED auto view switch");
    ctx_.oled_layout->next_view();
}

void NodeCallbacks::timeSyncTimerCallback(rcl_timer_t* timer, int64_t last_call_time) {
    ARG_UNUSED(last_call_time);

    if (!initialized_ || timer == NULL || ctx_.time_is_valid == nullptr) {
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
        atomic_set(ctx_.time_is_valid, 1);
    }
}

void NodeCallbacks::iim42652TimerCallback(rcl_timer_t* timer, int64_t last_call_time) {
    ARG_UNUSED(last_call_time);
    if (!initialized_ || timer == NULL || ctx_.iim42652_dev == NULL || !device_is_ready(ctx_.iim42652_dev) ||
        ctx_.iim42652_imu_publisher == nullptr || ctx_.iim42652_imu_msg == nullptr) {
        return;
    }

    struct sensor_value accel[3];
    struct sensor_value gyro[3];

    int rc = sensor_sample_fetch(ctx_.iim42652_dev);
    if (rc < 0) {
        LOG_ERR("IIM42652 sensor_sample_fetch error: %d", rc);
        return;
    }

    rc = sensor_channel_get(ctx_.iim42652_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
    if (rc < 0) {
        LOG_ERR("IIM42652 sensor_channel_get accel error: %d", rc);
        return;
    }

    rc = sensor_channel_get(ctx_.iim42652_dev, SENSOR_CHAN_GYRO_XYZ, gyro);
    if (rc < 0) {
        LOG_ERR("IIM42652 sensor_channel_get gyro error: %d", rc);
        return;
    }

    rcl_time_point_value_t now = rmw_uros_epoch_nanos();
    ctx_.iim42652_imu_msg->header.stamp.sec = now / 1000000000LL;
    ctx_.iim42652_imu_msg->header.stamp.nanosec = now % 1000000000LL;

    ctx_.iim42652_imu_msg->linear_acceleration.x = sensor_value_to_double(&accel[0]);
    ctx_.iim42652_imu_msg->linear_acceleration.y = sensor_value_to_double(&accel[1]);
    ctx_.iim42652_imu_msg->linear_acceleration.z = sensor_value_to_double(&accel[2]);

    ctx_.iim42652_imu_msg->angular_velocity.x = sensor_value_to_double(&gyro[0]);
    ctx_.iim42652_imu_msg->angular_velocity.y = sensor_value_to_double(&gyro[1]);
    ctx_.iim42652_imu_msg->angular_velocity.z = sensor_value_to_double(&gyro[2]);

    ctx_.iim42652_imu_msg->orientation_covariance[0] = -1.0;
    for (int i = 1; i < 9; ++i) {
        ctx_.iim42652_imu_msg->orientation_covariance[i] = 0.0;
        ctx_.iim42652_imu_msg->linear_acceleration_covariance[i] = 0.0;
        ctx_.iim42652_imu_msg->angular_velocity_covariance[i] = 0.0;
    }

    LOG_DBG("IIM42652 accel x=%.3f y=%.3f z=%.3f", ctx_.iim42652_imu_msg->linear_acceleration.x,
            ctx_.iim42652_imu_msg->linear_acceleration.y, ctx_.iim42652_imu_msg->linear_acceleration.z);

    RCSOFTCHECK(rcl_publish(ctx_.iim42652_imu_publisher, ctx_.iim42652_imu_msg, NULL));
}

void NodeCallbacks::lps22hbTimerCallback(rcl_timer_t* timer, int64_t last_call_time) {
    ARG_UNUSED(last_call_time);
    if (!initialized_ || timer == NULL || ctx_.st_lps22hb_dev == NULL || ctx_.lps22hb_temp_msg == nullptr ||
        ctx_.lps22hb_pressure_msg == nullptr || ctx_.lps22hb_temp_publisher == nullptr ||
        ctx_.lps22hb_pressure_publisher == nullptr || ctx_.oled_layout == nullptr || ctx_.storage == nullptr) {
        return;
    }

    struct sensor_value temp, press;
    int rc = sensor_sample_fetch(ctx_.st_lps22hb_dev);
    if (rc < 0) {
        LOG_ERR("LPS22HB sensor_sample_fetch error: %d", rc);
        return;
    }

    rc = sensor_channel_get(ctx_.st_lps22hb_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
    if (rc < 0) {
        LOG_ERR("LPS22HB sensor_channel_get ambient temp error: %d", rc);
        return;
    }

    rc = sensor_channel_get(ctx_.st_lps22hb_dev, SENSOR_CHAN_PRESS, &press);
    if (rc < 0) {
        LOG_ERR("LPS22HB sensor_channel_get pressure error: %d", rc);
        return;
    }

    rcl_time_point_value_t now = rmw_uros_epoch_nanos();
    ctx_.lps22hb_temp_msg->header.stamp.sec = now / 1000000000LL;
    ctx_.lps22hb_temp_msg->header.stamp.nanosec = now % 1000000000LL;
    ctx_.lps22hb_pressure_msg->header.stamp = ctx_.lps22hb_temp_msg->header.stamp;

    ctx_.lps22hb_temp_msg->temperature = sensor_value_to_double(&temp);
    ctx_.lps22hb_pressure_msg->fluid_pressure = sensor_value_to_double(&press) * 1000.0;

    RCSOFTCHECK(rcl_publish(ctx_.lps22hb_temp_publisher, ctx_.lps22hb_temp_msg, NULL));
    RCSOFTCHECK(rcl_publish(ctx_.lps22hb_pressure_publisher, ctx_.lps22hb_pressure_msg, NULL));

    char buffer[64];
    ctx_.oled_layout->display_temperature_pressure(temp.val1, temp.val2, press.val1, press.val2);
    memset(buffer, 0, sizeof(buffer));
    snprintf(buffer, sizeof(buffer), "T=%d.%02dC P=%d.%02dkPa", temp.val1, abs(temp.val2) / 10000, press.val1,
             abs(press.val2) / 10000);
    ctx_.storage->log_write(buffer);
    ctx_.oled_layout->finalize_screen();
}
