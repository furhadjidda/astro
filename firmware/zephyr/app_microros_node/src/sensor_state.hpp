#pragma once

#include <rcl/rcl.h>
#include <sensor_msgs/msg/fluid_pressure.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <sensor_msgs/msg/temperature.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

/* =========================================================
 * BNO055 IMU state
 * ========================================================= */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(bno055), okay)
struct Bno055State {
    const struct device* dev;
    bool fusion;
    rcl_publisher_t publisher;
    rcl_timer_t timer;
    sensor_msgs__msg__Imu msg;
};
#endif

/* =========================================================
 * IIM42652 IMU state
 * ========================================================= */

struct Iim42652State {
    const struct device* dev;
    rcl_publisher_t publisher;
    rcl_timer_t timer;
    sensor_msgs__msg__Imu msg;
};

/* =========================================================
 * LPS22HB barometer / thermometer state
 * ========================================================= */

struct Lps22hbState {
    const struct device* dev;
    rcl_publisher_t temp_publisher;
    rcl_publisher_t pressure_publisher;
    rcl_timer_t timer;
    sensor_msgs__msg__Temperature temp_msg;
    sensor_msgs__msg__FluidPressure pressure_msg;
};

/* =========================================================
 * GNSS state (shared layout for MTK3333 and u-blox)
 * ========================================================= */

#if DT_NODE_HAS_STATUS(DT_ALIAS(gnss), okay) || DT_NODE_HAS_STATUS(DT_ALIAS(ubloxgnss), okay)
struct GnssState {
    sensor_msgs__msg__NavSatFix msg;
    atomic_t msg_ready;  // bit 0: new data available
    struct k_spinlock msg_lock;
    atomic_t satellites_tracked;
    rcl_publisher_t publisher;
};
#endif
