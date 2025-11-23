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
#include "tof.hpp"

bool TOFSensor::initialize(i2c_inst_t *aI2cInstance) {
    if (!tofSensor->begin()) {
        printf("TOF sensor initialization failed!\n");
        return false;
    }
    printf("TOF sensor initialized successfully.\n");
    return true;
}

void TOFSensor::get_debug_message(std_msgs__msg__String &msg) {
    rosidl_runtime_c__String__assign(&msg.data, debug_msg_buffer);
}

void TOFSensor::get_tof_data(sensor_msgs__msg__Range &msg) {
    tofSensor->startRangeContinuous();
    if (tofSensor->isRangeComplete()) {
        uint16_t measure = tofSensor->readRange();

        // Set the radiation type (INFRARED for ToF sensors)
        msg.radiation_type = sensor_msgs__msg__Range__INFRARED;

        // Properly assign the frame_id string
        rosidl_runtime_c__String__assign(&msg.header.frame_id, "tof_frame");

        // Set the field of view, min range, and max range
        msg.field_of_view = 0.44;  // 25 degrees in radians
        msg.min_range = 0.03;      // Minimum measurable distance (meters)
        msg.max_range = 1.3;       // Maximum measurable distance (meters)

        // Convert the sensor reading to meters and assign to msg.range
        msg.range = measure / 1000.0;
    }
}