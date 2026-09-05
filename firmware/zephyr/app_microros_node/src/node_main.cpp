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
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/string_functions.h>
#include <sensor_msgs/msg/fluid_pressure.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <sensor_msgs/msg/temperature.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/display/cfb.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#if DT_NODE_HAS_STATUS(DT_NODELABEL(bno055), okay)
#include <bno055.h>
#endif
#if DT_NODE_HAS_STATUS(DT_ALIAS(gnss), okay) || DT_NODE_HAS_STATUS(DT_ALIAS(ubloxgnss), okay)
#include <zephyr/drivers/gnss.h>
#endif
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include "gnss_handler.hpp"
#include "microros_utils.hpp"
#include "node_callbacks.hpp"
#include "oled_layout.hpp"
#include "oled_wrapper.hpp"
#include "sensor_state.hpp"
#include "storage.hpp"
#include "wifi_handler.hpp"

/* Non-static: gnss_handler.cpp externs this to gate GNSS callbacks */
ATOMIC_DEFINE(init_complete, 1);

LOG_MODULE_REGISTER(all_sensors_module, LOG_LEVEL_DBG);

/* =========================================================
 * Shared subsystems
 * ========================================================= */

Storage storage;

static const struct device* const display_dev = DEVICE_DT_GET_ANY(solomon_ssd1306fb);
OLEDWrapper oled_wrapper(display_dev);
OLEDLayout oled_layout(&oled_wrapper);

static atomic_t time_is_valid;
static atomic_t oled_view_switch_request;

/* =========================================================
 * Per-sensor device pointers & state objects
 * ========================================================= */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(bno055), okay)
#define BNO055_TIMING_STARTUP 400
static const struct device* const bno055_dev = DEVICE_DT_GET(DT_NODELABEL(bno055));
static Bno055State bno055_state;
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(iim42652), okay)
#define IIM42652_TIMING_STARTUP 400
static const struct device* const iim42652_dev = DEVICE_DT_GET(DT_NODELABEL(iim42652));
#else
static const struct device* const iim42652_dev = NULL;
#endif
static Iim42652State iim42652_state;

static const struct device* const lps22hb_dev = DEVICE_DT_GET_ANY(st_lps22hb_press);
static Lps22hbState lps22hb_state;

/* =========================================================
 * micro-ROS shared objects
 * ========================================================= */

static rclc_support_t support;
static rcl_node_t node;
static rclc_executor_t executor;

static rcl_timer_t time_sync_timer;
static rcl_timer_t oled_view_timer;
#if DT_NODE_HAS_STATUS(DT_ALIAS(gnss), okay) || DT_NODE_HAS_STATUS(DT_ALIAS(ubloxgnss), okay)
static rcl_timer_t gnss_timer;
#endif

/* =========================================================
 * OLED view-switch button
 * ========================================================= */

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
 * micro-ROS executor thread
 * ========================================================= */

K_THREAD_STACK_DEFINE(executor_stack, EXECUTOR_STACK_SIZE);
static struct k_thread executor_thread;

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
 * Stage 1 — Hardware & sensor initialisation
 * ========================================================= */

static int init_hardware(void) {
#if DT_NODE_HAS_STATUS(DT_NODELABEL(bno055), okay)
#if Z_DEVICE_DT_FLAGS(DT_NODELABEL(bno055)) & DEVICE_FLAG_INIT_DEFERRED
    LOG_DBG("BNO055 deferred init, sleeping %d ms", BNO055_TIMING_STARTUP);
    k_sleep(K_MSEC(BNO055_TIMING_STARTUP));
    device_init(bno055_dev);
#endif
    if (!device_is_ready(bno055_dev)) {
        LOG_ERR("BNO055 device not ready");
        return -ENODEV;
    }
    bno055_state.dev = bno055_dev;
    bno055_state.fusion = true;
    struct sensor_value bno_cfg = {
        .val1 = bno055_state.fusion ? OPERATION_MODE_NDOF : OPERATION_MODE_M4G,
        .val2 = 0,
    };
    sensor_attr_set(bno055_dev, SENSOR_CHAN_ALL, SENSOR_ATTR_CONFIGURATION, &bno_cfg);
    bno_cfg.val1 = BNO055_POWER_NORMAL;
    bno_cfg.val2 = 0;
    sensor_attr_set(bno055_dev, SENSOR_CHAN_ALL, static_cast<sensor_attribute>(BNO055_SENSOR_ATTR_POWER_MODE),
                    &bno_cfg);
    sensor_msgs__msg__Imu__init(&bno055_state.msg);
    rosidl_runtime_c__String__assign(&bno055_state.msg.header.frame_id, "bno055_imu_frame");

    /* Delay between BNO055 and IIM42652 init to avoid I2C bus contention
     * and allow the first IMU to settle before the second is powered on. */
    k_msleep(1000);
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(iim42652), okay)
#if Z_DEVICE_DT_FLAGS(DT_NODELABEL(iim42652)) & DEVICE_FLAG_INIT_DEFERRED
    LOG_DBG("IIM42652 deferred init, sleeping %d ms", IIM42652_TIMING_STARTUP);
    k_sleep(K_MSEC(IIM42652_TIMING_STARTUP));
    device_init(iim42652_dev);
#endif
    /* Recovery: retry init if chip ID read failed (e.g. I2C bus not settled) */
    for (int _retry = 0; !device_is_ready(iim42652_dev) && _retry < 3; _retry++) {
        LOG_WRN("IIM42652 not ready (attempt %d/3), retrying in 500 ms...", _retry + 1);
        k_sleep(K_MSEC(500));
        device_init(iim42652_dev);
    }
    if (!device_is_ready(iim42652_dev)) {
        LOG_WRN("IIM42652 failed after retries, continuing without IMU");
    } else {
        LOG_DBG("IIM42652 ready: %s", iim42652_dev->name);
    }
    sensor_msgs__msg__Imu__init(&iim42652_state.msg);
    rosidl_runtime_c__String__assign(&iim42652_state.msg.header.frame_id, "iim42652_imu_frame");
#endif
    iim42652_state.dev = iim42652_dev;

    if (!device_is_ready(lps22hb_dev)) {
        LOG_ERR("LPS22HB device not ready");
        return -ENODEV;
    }
    lps22hb_state.dev = lps22hb_dev;
    sensor_msgs__msg__Temperature__init(&lps22hb_state.temp_msg);
    rosidl_runtime_c__String__assign(&lps22hb_state.temp_msg.header.frame_id, "lps22hb_frame");
    lps22hb_state.temp_msg.variance = 0.0;
    sensor_msgs__msg__FluidPressure__init(&lps22hb_state.pressure_msg);
    rosidl_runtime_c__String__assign(&lps22hb_state.pressure_msg.header.frame_id, "lps22hb_frame");
    lps22hb_state.pressure_msg.variance = 0.0;

#if DT_NODE_HAS_STATUS(DT_ALIAS(gnss), okay)
    sensor_msgs__msg__NavSatFix__init(&mtk3333_state.msg);
    rosidl_runtime_c__String__assign(&mtk3333_state.msg.header.frame_id, "mtk3333_gnss_frame");
#endif
#if DT_NODE_HAS_STATUS(DT_ALIAS(ubloxgnss), okay)
    sensor_msgs__msg__NavSatFix__init(&ublox_state.msg);
    rosidl_runtime_c__String__assign(&ublox_state.msg.header.frame_id, "ublox_gnss_frame");
#endif

    int oled_init_rc = oled_layout.init_screen();
    if (oled_init_rc != 0) {
        LOG_WRN("OLED unavailable (%d), continuing without display", oled_init_rc);
    }
    storage.init();
    storage.print_storage_stats();

#if DT_NODE_HAS_STATUS(DT_ALIAS(gnss), okay)
    {
#define GNSS_SYSTEMS_PRINTF(define, supported, enabled)                                                      \
    LOG_DBG("\t%20s: Supported: %3s Enabled: %3s\n", STRINGIFY(define), (supported & define) ? "Yes" : "No", \
            (enabled & define) ? "Yes" : "No")
        gnss_systems_t supported, enabled;
        uint32_t fix_interval;
        int rc = gnss_get_supported_systems(mtk3333_gnss, &supported);
        if (rc < 0) {
            LOG_ERR("Failed to query supported systems (%d)", rc);
            return rc;
        }
        rc = gnss_get_enabled_systems(mtk3333_gnss, &enabled);
        if (rc < 0) {
            LOG_ERR("Failed to query enabled systems (%d)", rc);
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
            LOG_ERR("Failed to query fix rate (%d)", rc);
            return rc;
        }
        LOG_DBG("Fix rate = %d ms", fix_interval);
    }
#endif

    return 0;
}

/* =========================================================
 * Stage 2 — Transport & agent connection
 * ========================================================= */

static int init_transport(void) {
    k_sleep(K_MSEC(10)); /* allow rail to stabilize */

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
        agent_wait_round++;
        if ((agent_wait_round % 5U) == 0U) {
            LOG_WRN("micro-ROS agent still unreachable (%u checks)", agent_wait_round);
        }
        k_sleep(K_MSEC(500));
    }
    LOG_INF("micro-ROS agent connected");

    if (oled_wrapper.is_available()) {
        oled_layout.show_startup_splash();
        k_sleep(K_MSEC(1800));
        oled_layout.set_display_updates_enabled(true);
        oled_layout.clear_screen();
        oled_layout.set_view(OLEDLayout::View::SATELLITE);
        atomic_clear(&oled_view_switch_request);
        (void)init_view_switch_button();
    }

    return 0;
}

/* =========================================================
 * Stage 3 — micro-ROS node, publishers, timers, executor
 * ========================================================= */

static int init_microros_node(void) {
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, 10));
    LOG_INF("micro-ROS using ROS domain ID 10");

    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    node = rcl_get_zero_initialized_node();
    RCCHECK(rclc_node_init_default(&node, "sensor_publisher", "", &support));

    /* Publishers — GNSS reliable for rqt compat; IMU best-effort to avoid backpressure */
#if DT_NODE_HAS_STATUS(DT_ALIAS(gnss), okay)
    RCCHECK(rclc_publisher_init_default(&mtk3333_state.publisher, &node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix), "/mtk3333_gnss_raw"));
#endif
#if DT_NODE_HAS_STATUS(DT_ALIAS(ubloxgnss), okay)
    RCCHECK(rclc_publisher_init_default(&ublox_state.publisher, &node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix), "/ublox_gnss_raw"));
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(bno055), okay)
    RCCHECK(rclc_publisher_init_default(&bno055_state.publisher, &node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/bno055_imu_raw"));
#endif
    RCCHECK(rclc_publisher_init_default(&lps22hb_state.temp_publisher, &node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature),
                                        "/lps22hb_temperature_raw"));
    RCCHECK(rclc_publisher_init_default(&lps22hb_state.pressure_publisher, &node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, FluidPressure),
                                        "/lps22hb_pressure_raw"));
#if DT_NODE_HAS_STATUS(DT_NODELABEL(iim42652), okay)
    RCCHECK(rclc_publisher_init_default(&iim42652_state.publisher, &node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/iim42652_imu_raw"));
#endif

    /* Wire callback context */
    NodeCallbacks::Context ctx = {};
    ctx.oled_layout = &oled_layout;
    ctx.storage = &storage;
    ctx.time_is_valid = &time_is_valid;
    ctx.oled_view_switch_request = &oled_view_switch_request;
#if DT_NODE_HAS_STATUS(DT_NODELABEL(bno055), okay)
    ctx.bno055 = &bno055_state;
#endif
#if DT_NODE_HAS_STATUS(DT_ALIAS(gnss), okay)
    ctx.mtk3333 = &mtk3333_state;
#endif
#if DT_NODE_HAS_STATUS(DT_ALIAS(ubloxgnss), okay)
    ctx.ublox = &ublox_state;
#endif
    ctx.iim42652 = &iim42652_state;
    ctx.lps22hb = &lps22hb_state;
    NodeCallbacks::initialize(ctx);

    /* Timers */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(bno055), okay)
    RCCHECK(rclc_timer_init_default(&bno055_state.timer, &support, RCL_MS_TO_NS(IMU_PUBLISH_PERIOD_MS),
                                    NodeCallbacks::bno055ImuTimerCallback));
#endif
#if DT_NODE_HAS_STATUS(DT_ALIAS(gnss), okay) || DT_NODE_HAS_STATUS(DT_ALIAS(ubloxgnss), okay)
    RCCHECK(rclc_timer_init_default(&gnss_timer, &support, RCL_MS_TO_NS(GNSS_PUBLISH_PERIOD_MS),
                                    NodeCallbacks::gnssPublishTimerCallback));
#endif
    RCCHECK(rclc_timer_init_default(&time_sync_timer, &support, RCL_MS_TO_NS(TIME_SYNC_PERIOD_MS),
                                    NodeCallbacks::timeSyncTimerCallback));
    RCCHECK(rclc_timer_init_default(&lps22hb_state.timer, &support, RCL_MS_TO_NS(LPS22HB_PUBLISH_PERIOD_MS),
                                    NodeCallbacks::lps22hbTimerCallback));
    RCCHECK(rclc_timer_init_default(&oled_view_timer, &support, RCL_MS_TO_NS(OLED_VIEW_SWITCH_PERIOD_MS),
                                    NodeCallbacks::oledViewTimerCallback));
#if DT_NODE_HAS_STATUS(DT_NODELABEL(iim42652), okay)
    RCCHECK(rclc_timer_init_default(&iim42652_state.timer, &support, RCL_MS_TO_NS(IIM42652_PUBLISH_PERIOD_MS),
                                    NodeCallbacks::iim42652TimerCallback));
#endif

    /* Executor */
    RCCHECK(rclc_executor_init(&executor, &support.context, 6, &allocator));
#if DT_NODE_HAS_STATUS(DT_NODELABEL(bno055), okay)
    RCCHECK(rclc_executor_add_timer(&executor, &bno055_state.timer));
#endif
#if DT_NODE_HAS_STATUS(DT_ALIAS(gnss), okay) || DT_NODE_HAS_STATUS(DT_ALIAS(ubloxgnss), okay)
    RCCHECK(rclc_executor_add_timer(&executor, &gnss_timer));
#endif
    RCCHECK(rclc_executor_add_timer(&executor, &time_sync_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &lps22hb_state.timer));
    RCCHECK(rclc_executor_add_timer(&executor, &oled_view_timer));
#if DT_NODE_HAS_STATUS(DT_NODELABEL(iim42652), okay)
    RCCHECK(rclc_executor_add_timer(&executor, &iim42652_state.timer));
#endif

    return 0;
}

/* =========================================================
 * main()
 * ========================================================= */

int main(void) {
    LOG_DBG("Astro sensor node starting");

    if (init_hardware() != 0) {
        LOG_ERR("Hardware init failed");
        return -1;
    }
    if (init_transport() != 0) {
        LOG_ERR("Transport init failed");
        return -1;
    }
    if (init_microros_node() != 0) {
        LOG_ERR("micro-ROS node init failed");
        return -1;
    }

    k_thread_create(&executor_thread, executor_stack, EXECUTOR_STACK_SIZE, executor_thread_entry, NULL, NULL, NULL,
                    EXECUTOR_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&executor_thread, "uros_executor");
    LOG_DBG("micro-ROS threads started");

    atomic_set_bit(init_complete, 0);  // open the gate for GNSS callbacks

    while (1) {
        k_sleep(K_FOREVER);
    }
}
