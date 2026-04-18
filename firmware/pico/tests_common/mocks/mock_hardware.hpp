/*
 *   This file is part of embedded software pico playground project.
 *
 *   embedded software pico playground projec is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   embedded software pico playground project is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License v3.0
 *   along with embedded software pico playground project.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef MOCK_HARDWARE_H
#define MOCK_HARDWARE_H

#include <cstdint>

#include "mock_i2c.hpp"
typedef mock_i2c_inst_t i2c_inst_t;

inline void i2c_init(i2c_inst_t* i2c, uint32_t baudrate) {
    // Mock implementation
}

inline void gpio_set_function(uint pin, int function) {
    // Mock implementation
}

inline void gpio_pull_up(uint pin) {
    // Mock implementation
}

inline uint64_t time_us_64() {
    // Mock implementation
    return 0;
}

#define GPIO_FUNC_I2C 4  // Mock definition for GPIO function I2C

#endif  // MOCK_HARDWARE_H
