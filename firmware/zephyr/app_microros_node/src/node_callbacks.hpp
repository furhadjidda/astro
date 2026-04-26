#pragma once

#include <rcl/rcl.h>
#include <sensor_msgs/msg/fluid_pressure.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <sensor_msgs/msg/temperature.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

class OLEDLayout;
class Storage;

class NodeCallbacks {
   public:
    struct Context {
        OLEDLayout* oled_layout;
        Storage* storage;
        atomic_t* time_is_valid;
        atomic_t* oled_view_switch_request;

#if DT_NODE_HAS_STATUS(DT_NODELABEL(bno055), okay)
        const struct device* bno055_dev;
        rcl_publisher_t* bno055_imu_publisher;
        sensor_msgs__msg__Imu* bno055_imu_msg;
#endif

#if DT_NODE_HAS_STATUS(DT_ALIAS(gnss), okay)
        rcl_publisher_t* mtk3333_gnss_publisher;
        sensor_msgs__msg__NavSatFix* mtk3333_nav_sat_fix_msg;
        atomic_t* mtk3333_msg_ready;
        struct k_spinlock* mtk3333_msg_lock;
#endif

#if DT_NODE_HAS_STATUS(DT_ALIAS(ubloxgnss), okay)
        rcl_publisher_t* ublox_gnss_publisher;
        sensor_msgs__msg__NavSatFix* ublox_nav_sat_fix_msg;
        atomic_t* ublox_msg_ready;
        struct k_spinlock* ublox_msg_lock;
#endif

        const struct device* iim42652_dev;
        rcl_publisher_t* iim42652_imu_publisher;
        sensor_msgs__msg__Imu* iim42652_imu_msg;

        const struct device* st_lps22hb_dev;
        rcl_publisher_t* lps22hb_temp_publisher;
        rcl_publisher_t* lps22hb_pressure_publisher;
        sensor_msgs__msg__Temperature* lps22hb_temp_msg;
        sensor_msgs__msg__FluidPressure* lps22hb_pressure_msg;
    };

    static void initialize(const Context& context);
    static void handleExecutorIteration();

#if DT_NODE_HAS_STATUS(DT_NODELABEL(bno055), okay)
    static void bno055ImuTimerCallback(rcl_timer_t* timer, int64_t last_call_time);
#endif

#if DT_NODE_HAS_STATUS(DT_ALIAS(gnss), okay) || DT_NODE_HAS_STATUS(DT_ALIAS(ubloxgnss), okay)
    static void gnssPublishTimerCallback(rcl_timer_t* timer, int64_t last_call_time);
#endif

    static void oledViewTimerCallback(rcl_timer_t* timer, int64_t last_call_time);
    static void timeSyncTimerCallback(rcl_timer_t* timer, int64_t last_call_time);
    static void iim42652TimerCallback(rcl_timer_t* timer, int64_t last_call_time);
    static void lps22hbTimerCallback(rcl_timer_t* timer, int64_t last_call_time);

   private:
    static Context ctx_;
    static bool initialized_;
};
