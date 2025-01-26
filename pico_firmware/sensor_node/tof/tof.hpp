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
#ifndef TOF_HPP
#define TOF_HPP

#include <rosidl_runtime_c/string_functions.h>
#include <sensor_msgs/msg/range.h>
#include <std_msgs/msg/string.h>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "Adafruit_VL53L0X.h"
#include "rcl/time.h"
#include "rcutils/time.h"
#include "sensor.hpp"

class TOFSensor : public Sensor {
   public:
    bool initialize(i2c_inst_t *aI2cInstance) override;
    void get_tof_data(sensor_msgs__msg__Range &msg);
    virtual void get_debug_message(std_msgs__msg__String &msg) override;

   private:
    std::unique_ptr<Adafruit_VL53L0X> tofSensor = std::make_unique<Adafruit_VL53L0X>();
    char debug_msg_buffer[1024];
};

#endif  // TOF_HPP