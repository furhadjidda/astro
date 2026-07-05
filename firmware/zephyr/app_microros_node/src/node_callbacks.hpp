#pragma once

#include <rcl/rcl.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "sensor_state.hpp"

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
        Bno055State* bno055;
#endif

#if DT_NODE_HAS_STATUS(DT_ALIAS(gnss), okay)
        GnssState* mtk3333;
#endif

#if DT_NODE_HAS_STATUS(DT_ALIAS(ubloxgnss), okay)
        GnssState* ublox;
#endif

        Iim42652State* iim42652;
        Lps22hbState* lps22hb;
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
