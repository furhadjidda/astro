/*
 *   This file is part of the astro project.
 *
 *   Astro project is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   Astro project is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License v3.0
 *   along with astro project.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "gnss_parser.hpp"
#include "hardware/i2c.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "pico_uart_transports.h"
#include <Adafruit_GPS.hpp>
#include <memory>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/string_functions.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <sensor_msgs/msg/nav_sat_status.h>
#include <std_msgs/msg/string.h>
#include <string>

// ROS Publishers
rcl_publisher_t publisher;
rcl_publisher_t dbg_publisher;
uint32_t timer = millis();

#define PUBLISH_RATE_HZ 5

// GPS parser and device
std::unique_ptr<gnss_parser> parser = std::make_unique<gnss_parser>();
std::unique_ptr<Adafruit_GPS> GPS = std::make_unique<Adafruit_GPS>(i2c0);

// Messages
sensor_msgs__msg__NavSatFix nav_sat_fix_msg;
std_msgs__msg__String dbg_msg;

// Constants
#define GPSECHO false
#define GPS_I2C_ADDRESS 0x10

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

void setup_gps() {
    if (!GPS->Init(GPS_I2C_ADDRESS)) {
        publish_debug_message("Error initializing GPS\n");
        return;
    }

    // Configure GPS
    GPS->SendCommand(reinterpret_cast<const uint8_t *>(PMTK_SET_NMEA_OUTPUT_ALLDATA),
                     strlen(PMTK_SET_NMEA_OUTPUT_ALLDATA));
    GPS->SendCommand(reinterpret_cast<const uint8_t *>(PMTK_SET_NMEA_UPDATE_1HZ), strlen(PMTK_SET_NMEA_UPDATE_1HZ));
    GPS->SendCommand(reinterpret_cast<const uint8_t *>(PGCMD_ANTENNA), strlen(PGCMD_ANTENNA));
    // sleep_ms(1000);
}

void process_gps_data() {
    char c = GPS->ReadData();

    if (GPS->NewNMEAreceived()) {
        std::string nmea_sentence(GPS->LastNMEA());
        if (!GPS->Parse(GPS->LastNMEA())) {
            return;
        }

        // Publish debug message
        std::string fix_status = (GPS->mFix) ? "true" : "false";
        publish_debug_message("Received NMEA sentence: " + nmea_sentence + " >>> Fix: " + fix_status);
        // Pack and publish GNSS data
        nav_sat_fix_msg = parser->packData(GPS->mLatitude, GPS->mLat, GPS->mLongitude, GPS->mLon, GPS->mAltitude,
                                           GPS->mFix, GPS->mFixquality_3d, GPS->mHDOP, GPS->mPDOP, GPS->mVDOP);
        rcl_time_point_value_t now;
        rcutils_time_point_value_t time_now;
        rcutils_system_time_now(&time_now);
        now = static_cast<rcl_time_point_value_t>(time_now);
        // Populate the timestamp
        nav_sat_fix_msg.header.stamp.sec = static_cast<uint32_t>(now / RCL_S_TO_NS(1));
        nav_sat_fix_msg.header.stamp.nanosec = static_cast<uint32_t>(now % RCL_S_TO_NS(1));
        rcl_publish(&publisher, &nav_sat_fix_msg, NULL);
    }
    if (millis() - timer > 2000) {
        timer = millis();
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, !cyw43_arch_gpio_get(CYW43_WL_GPIO_LED_PIN));
    }
}

int main() {
    // Initialize micro-ROS transport
    rmw_uros_set_custom_transport(true, NULL, pico_serial_transport_open, pico_serial_transport_close,
                                  pico_serial_transport_write, pico_serial_transport_read);

    // Initialize GPS
    setup_gps();
    cyw43_arch_init();
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    // Initialize micro-ROS components
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_support_init(&support, 0, NULL, &allocator);

    rcl_node_t node;
    rclc_node_init_default(&node, "gnss_publisher", "", &support);

    rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix),
                                "gnss_data");
    rclc_publisher_init_default(&dbg_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "dbg_msg");

    rclc_executor_t executor;
    rclc_executor_init(&executor, &support.context, 1, &allocator);

    nav_sat_fix_msg.status.service = sensor_msgs__msg__NavSatStatus__SERVICE_GPS;

    while (true) {
        process_gps_data();
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }

    // Cleanup
    rcl_publisher_fini(&publisher, &node);
    rcl_publisher_fini(&dbg_publisher, &node);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
    rclc_executor_fini(&executor);

    return 0;
}
