/*
 *   This file is part of astro_core_ros.
 *
 *   astro_core_ros is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   astro_core_ros is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License v3.0
 *   along with astro_core_ros.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef __COMMON_HPP__
#define __COMMON_HPP__

enum MotorId {
    MotorA = 0,
    MotorB,
    MaxMotorValue
};

enum MotorDirection {
    Forward = 0,
    Backward,
    MotorDirectionMaxValue
};

// Motor Characteristics
static const uint16_t ppr = 1176; // pulses/ticks per revolution , gear Ration = 1/98 and each loop output pulses = 12 PPR. Hence 98*12 = 1176. Also called as resolution
static const uint8_t cpr = 48; // counts per revolution
static const uint8_t wheelRadius = 39; // 78 mm diameter

const float RpmToRadians = 0.10471975512;
const float RadiansToDegree = 57.29578;

#endif