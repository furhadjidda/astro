/*
 *   This file is part of astro project.
 *
 *   astro projec is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   astro project is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License v3.0
 *   along with astro project.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <sensor_msgs/msg/nav_sat_status.h>
#include <std_msgs/msg/string.h>  // Include this for debug messages
#include <hardware/uart.h>
#include <pico/stdlib.h>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <Adafruit_GPS.hpp>
#include <rmw_microros/rmw_microros.h>
#include "pico/stdlib.h"
#include "pico_uart_transports.h"
#include "gnss_parser.hpp"

#define UART_ID uart0
#define BAUD_RATE 9600
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define BUFFER_SIZE 256

static char nmea_buffer[BUFFER_SIZE];
static size_t nmea_buffer_index = 0;

rcl_publisher_t publisher;
rcl_publisher_t dbg_publisher;  // Debug publisher
gnss_parser parser;
sensor_msgs__msg__NavSatFix nav_sat_fix_msg;
std::string dbg_msg;  // Message type for debug log

// Connect to the GPS on the hardware I2C port
Adafruit_GPS GPS(i2c0);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
#define GPSECHO false

uint32_t timer = millis();

void setup() {
    GPS.Init(0x10);  // The I2C address to use is 0x10

    // Set GPS parameters
    GPS.SendCommand(reinterpret_cast<const uint8_t*>(PMTK_SET_NMEA_OUTPUT_ALLDATA),
                    strlen(PMTK_SET_NMEA_OUTPUT_ALLDATA));
    GPS.SendCommand(reinterpret_cast<const uint8_t*>(PMTK_SET_NMEA_UPDATE_1HZ),
                    strlen(PMTK_SET_NMEA_UPDATE_1HZ));
    GPS.SendCommand(reinterpret_cast<const uint8_t*>(PGCMD_ANTENNA), strlen(PGCMD_ANTENNA));
    sleep_ms(1000);
}

void loop() {
    // Read data from the GPS in the 'main loop'
    char c = GPS.ReadData();
    if (GPS.NewNMEAreceived()) {
        std::string nmea_sentence(GPS.LastNMEA());
        if (!GPS.Parse(GPS.LastNMEA())) {
            return;
        }

        // Publish debug message
        std::string fix = (GPS.mFix) ? "true" : "false";
        dbg_msg = std::string("Received NMEA sentence: " + nmea_sentence + " >>> Fix " + fix );  // Set debug message
        rcl_publish(&dbg_publisher, &dbg_msg, NULL);  // Publish to dbg_mesg topic


        nav_sat_fix_msg = parser.packData(GPS.mLatitude,GPS.mLat, GPS.mLongitude,GPS.mLon ,GPS.mAltitude, GPS.mFix,GPS.mFixquality_3d,GPS.mHDOP,GPS.mPDOP,GPS.mVDOP );
        rcl_publish(&publisher, &nav_sat_fix_msg, NULL);
    }
}

int main() {
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    setup();

    // Initialize micro-ROS
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_support_init(&support, 0, NULL, &allocator);

    rcl_node_t node;
    rclc_node_init_default(&node, "gnss_publisher", "", &support);

    rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix), "gnss_data");
    rclc_publisher_init_default(&dbg_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "dbg_mesg");  // Initialize debug publisher

    rclc_executor_t executor;
    rclc_executor_init(&executor, &support.context, 1, &allocator);

    nav_sat_fix_msg.status.service = sensor_msgs__msg__NavSatStatus__SERVICE_GPS;

    while (true) {
        loop();
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }

    // Cleanup
    rcl_publisher_fini(&publisher, &node);
    rcl_publisher_fini(&dbg_publisher, &node);  // Cleanup debug publisher
    rcl_node_fini(&node);
    rclc_support_fini(&support);

    return 0;
}
