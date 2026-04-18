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
#ifndef SENSOR_HPP
#define SENSOR_HPP

#include <rosidl_runtime_c/string_functions.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <sensor_msgs/msg/nav_sat_status.h>
#include <sensor_msgs/msg/range.h>
#include <std_msgs/msg/string.h>

#include "hardware/i2c.h"
#include "pico/stdlib.h"
class Sensor {
   public:
    virtual ~Sensor() = default;
    virtual bool initialize(i2c_inst_t* aI2cInstance) = 0;
    virtual void get_debug_message(std_msgs__msg__String& msg) = 0;
    // Add virtual methods for specific sensor types
    virtual void get_imu_data(sensor_msgs__msg__Imu& msg) {}

    virtual void get_tof_data(sensor_msgs__msg__Range& msg) {}

    virtual void get_gnss_data(sensor_msgs__msg__NavSatFix& msg) {}

    virtual void create_welcome_screen() {}
};

#endif  // SENSOR_HPP