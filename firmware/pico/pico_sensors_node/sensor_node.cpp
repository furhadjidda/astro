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

#include <math.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/string.h>
#include <std_srvs/srv/trigger.h>

#include "display.hpp"
#include "gnss.hpp"
#include "hardware/i2c.h"
#include "hardware/watchdog.h"
#include "imu.hpp"
#include "pico/cyw43_arch.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "pico_uart_transports.h"
#include "rcl/time.h"
#include "rcutils/time.h"
#include "sensor.hpp"
#include "sensorFactory.hpp"
#include "tof.hpp"
extern "C" {
#include "pico/bootrom.h"
}

#define PUBLISH_RATE_HZ 10
#define DEBUG_PUBLISH_RATE_HZ 1
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__Range tof_range_msg;
sensor_msgs__msg__NavSatFix nav_sat_fix_msg;
std_msgs__msg__String dbg_msg;

// ROS Publishers
rcl_publisher_t imu_publisher;
rcl_publisher_t dbg_publisher;
rcl_publisher_t tof_publisher;
rcl_publisher_t gnss_publisher;

// ROS subscriber
rcl_subscription_t imu_subscriber;

// Objects
static std::unique_ptr<Sensor> imuSensor;
static std::unique_ptr<Sensor> tofSensor;
static std::unique_ptr<Sensor> gnssSensor;
static std::unique_ptr<Sensor> displaySensor;

std_srvs__srv__Trigger_Request reboot_req;
std_srvs__srv__Trigger_Response reboot_resp;

// watchdog variables
volatile uint32_t last_msg_time = 0;
#define WATCHDOG_TIMEOUT_MS 10000  // 10 seconds timeout

std_msgs__msg__String debug_message;

// Helper function to register sensors
void registerSensors() {
    SensorFactory::getInstance().registerSensor("IMU", []() { return std::make_unique<IMUSensor>(); });
    SensorFactory::getInstance().registerSensor("GNSS", []() { return std::make_unique<GNSSSensor>(); });
    SensorFactory::getInstance().registerSensor("TOF", []() { return std::make_unique<TOFSensor>(); });
    SensorFactory::getInstance().registerSensor("DISPLAY", []() { return std::make_unique<DisplaySensor>(); });
}

// Publish Debug data
void publish_debug_message(const std::string &message) {
    // Ensure the message memory is managed correctly
    if (!rosidl_runtime_c__String__assign(&dbg_msg.data, message.c_str())) {
        // Handle error
        printf("Failed to assign debug message\n");
        return;
    }

    // Publish the debug message
    if (RCL_RET_OK != rcl_publish(&dbg_publisher, &dbg_msg, NULL)) {
        printf("Failed to publish debug message\n");
    }
}

void imu_callback(const void *msgin) {
    last_msg_time = to_ms_since_boot(get_absolute_time());
    publish_debug_message("IMU message received: Resetting watchdog timer");
}

// This uses core1 to handle time synchronization
void core1_time_sync_loop() {
    while (true) {
        bool ok = rmw_uros_sync_session(50);  // small timeout
        if (!ok) {
            printf("Time sync failed\n");
        }
        sleep_ms(1000);  // sync every 1 second (or whatever interval you want)
    }
}

void reboot_callback(const void *req_msg, void *resp_msg) {
    (void)req_msg;  // unused

    auto response = (std_srvs__srv__Trigger_Response *)resp_msg;
    response->success = true;
    strcpy(response->message.data, "Rebooting into bootloader...");

    sleep_ms(1000);        // allow ROS message to flush
    reset_usb_boot(0, 0);  // reboot Pico into UF2 bootloader
}

int main() {
    setenv("ROS_DOMAIN_ID", "10", 1);
    rmw_uros_set_custom_transport(true, NULL, pico_serial_transport_open, pico_serial_transport_close,
                                  pico_serial_transport_write, pico_serial_transport_read);

    // Enable Watchdog
    watchdog_enable(WATCHDOG_TIMEOUT_MS, 1);

    registerSensors();
    imuSensor = SensorFactory::getInstance().createSensor("IMU");
    gnssSensor = SensorFactory::getInstance().createSensor("GNSS");
    tofSensor = SensorFactory::getInstance().createSensor("TOF");
    displaySensor = SensorFactory::getInstance().createSensor("DISPLAY");
    imuSensor->initialize(i2c0);
    gnssSensor->initialize(i2c0);
    tofSensor->initialize(i2c0);
    displaySensor->initialize(i2c0);

    // This is for LED on pico
    cyw43_arch_init();
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    rmw_uros_ping_agent(1000, 10);
    if (!rmw_uros_sync_session(2000)) {
        printf("Time sync failed\n");
    } else {
        printf("Time sync OK\n");
    }
    // enables core1 and starts time sync loop
    multicore_launch_core1(core1_time_sync_loop);

    // Initialize micro-ROS
    rcl_allocator_t allocator = rcl_get_default_allocator();
    // Initialize and modify options (Set DOMAIN ID to 10)
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, 10);

    // Initialize rclc support object with custom option
    rclc_support_t support;
    rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

    rcl_node_t node;
    // initialize Node
    rclc_node_init_default(&node, "imu_gnss_publisher", "", &support);
    // initialize publishers
    rclc_publisher_init_default(&imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu_raw");
    rclc_publisher_init_default(&tof_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
                                "/tof_data_raw");
    rclc_publisher_init_default(&gnss_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix),
                                "/gnss_data_raw");
    rclc_publisher_init_default(&dbg_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "dbg_msg");

    // Initialize subscribers
    // Initialize IMU subscriber
    rclc_subscription_init_default(&imu_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
                                   "/imu_raw");

    // Service initializations
    rcl_service_t service;

    rclc_service_init_default(&service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
                              "reboot_to_bootloader");

    rcl_timer_t timer;
    rcl_timer_t debug_timer;
    //  initialize timers
    rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(1000 / PUBLISH_RATE_HZ),
                            [](rcl_timer_t *timer, int64_t last_call_time) {
                                imuSensor->get_imu_data(imu_msg);
                                rcl_publish(&imu_publisher, &imu_msg, NULL);
                                tofSensor->get_tof_data(tof_range_msg);
                                rcl_publish(&tof_publisher, &tof_range_msg, NULL);
                                gnssSensor->get_gnss_data(nav_sat_fix_msg);
                                rcl_publish(&gnss_publisher, &nav_sat_fix_msg, NULL);
                                cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, !cyw43_arch_gpio_get(CYW43_WL_GPIO_LED_PIN));
                            });

    // initialize timers
    rclc_timer_init_default(&debug_timer, &support, RCL_MS_TO_NS(1000 / DEBUG_PUBLISH_RATE_HZ),
                            [](rcl_timer_t *timer, int64_t last_call_time) {
                                imuSensor->get_debug_message(debug_message);
                                rcl_publish(&dbg_publisher, &debug_message, NULL);
                            });

    rclc_executor_t executor;
    // initialize executor
    rclc_executor_init(&executor, &support.context, 4, &allocator);

    // add the timer to the executor
    rclc_executor_add_timer(&executor, &timer);
    rclc_executor_add_timer(&executor, &debug_timer);

    // Assign callback
    rclc_executor_add_subscription(&executor, &imu_subscriber, &imu_msg, &imu_callback, ON_NEW_DATA);
    rclc_executor_add_service(&executor, &service, &reboot_req, &reboot_resp, &reboot_callback);
    displaySensor->create_welcome_screen();
    last_msg_time = to_ms_since_boot(get_absolute_time());
    // Main loop spin
    while (rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)) == RCL_RET_OK) {
        if (to_ms_since_boot(get_absolute_time()) - last_msg_time > WATCHDOG_TIMEOUT_MS) {
            printf("Watchdog triggered: No IMU data received, resetting!\n");
            watchdog_reboot(0, 0, 0);
        }
        // Reset watchdog timer
        watchdog_update();
    }

    // Clean up
    rcl_publisher_fini(&imu_publisher, &node);
    rclc_support_fini(&support);
    rcl_publisher_fini(&gnss_publisher, &node);
    rcl_publisher_fini(&dbg_publisher, &node);
    rcl_publisher_fini(&tof_publisher, &node);
    rcl_node_fini(&node);
    rcl_timer_fini(&timer);
    rcl_timer_fini(&debug_timer);
    rclc_support_fini(&support);
    rclc_executor_fini(&executor);
    rcl_subscription_fini(&imu_subscriber, &node);

    return 0;
}
