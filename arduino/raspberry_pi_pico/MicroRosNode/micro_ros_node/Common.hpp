#include <sys/_types.h>
#include <sys/_intsup.h>
#include <sys/_stdint.h>

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

#include "Arduino.h"
#include "mbed.h"
#include <Timer.h>
#include <chrono>
using namespace std::chrono;

enum MotorId {
    MotorA = 0, // Right
    MotorB, // Left
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
static const uint8_t wheelDistance = 182; // distance between the wheels

const float RpmToRadians = 0.10471975512;
const float RadiansToDegree = 57.29578;
const double Kp = 1.0;
const double Ki = 0.0;
const double Kd = 0.0; // PID constants

/* Global parameters */
static const unsigned int FREQUENCY_RATE = 30; 			    // [ms] default 50ms
static const unsigned int FREQUENCY_ODOMETRY = 150; 		// [ms] default 250ms
static const unsigned int FREQUENCY_CONTROLLER 	= 30; 	// [ms] default 50ms


/* Rate computing parameters */
#define RATE_DIRECTION_MEDIAN_FILTER_SIZE 	3
#define RATE_CONV 							0.005343197 // 0.0073882 	// [inc] -> [rad]
// 853 (234) inc per wheel rotation
// RATE_CONV = 2*pi/(Nb_increments_per_wheel_rotation) = 2*pi/299
// In my case this is close to 299 which is why the value 0.021
#define RATE_AVERAGE_FILTER_SIZE 			4

/* Rate controller parameters */
#define PWM_MAX				 				4095            // 12 bit
#define RATE_CONTROLLER_KP 					130.0 		    // default 150
#define RATE_CONTROLLER_KD 					5000000000000.0 //4500000000000
#define RATE_CONTROLLER_KI 					0.00005 	    //0.00001
#define RATE_INTEGRAL_FREEZE				250
#define RATE_CONTROLLER_MIN_PWM 			-500
#define RATE_CONTROLLER_MAX_PWM 			500

/* Mechanical parameters */
#define WHEEL_RADIUS 						0.039 		// [m]
// distance between the two wheels
#define BASE_LENGTH 						0.182 		// [m]

#endif