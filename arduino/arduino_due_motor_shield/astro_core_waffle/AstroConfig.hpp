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

#ifndef ASTRO_CONFIG_H_
#define ASTRO_CONFIG_H_

// enable this line for Arduino Due
#define USE_USBCON
// Type of ROS Platform
#define ASTRO_WAFFLE

/* Include librairies */

#include <Arduino.h>

/* Global parameters */
#define FREQUENCY_RATE 						30 			// [ms] default 50ms
#define FREQUENCY_ODOMETRY 				    150 		// [ms] default 250ms
#define FREQUENCY_ROSPINONCE 				150 		// [ms]
#define FREQUENCY_CONTROLLER 				30 			// [ms] default 50ms

/* Rate computing parameters */
#define RATE_DIRECTION_MEDIAN_FILTER_SIZE 	3
#define RATE_CONV 							0.021 // 0.0073882 	// [inc] -> [rad]
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

// If nothing is selected then by default Burger will be selected
#if defined(ASTRO_BURGER)
    /* Mechanical parameters */
    #define WHEEL_RADIUS 0.036 // [m]
    // distance between the two wheels
    #define BASE_LENGTH 0.140 // [m]  0.288
#elif defined(ASTRO_WAFFLE)
    /* Mechanical parameters */
    #define WHEEL_RADIUS 						0.0376 		// [m]
    // distance between the two wheels
    #define BASE_LENGTH 						0.272 		// [m]  0.288
#else
    /* Mechanical parameters */
    #define WHEEL_RADIUS 0.036 // [m]
    // distance between the two wheels
    #define BASE_LENGTH 0.140 // [m]  0.288
#endif


/* Define pins */
// motor A (right)
const byte motorRightEncoderPinA = 26; // 38
const byte motorRightEncoderPinB = 30; // 34
const byte enMotorRight = 3; // 2 ==> speed
const byte in1MotorRight = 12;   //26 C1 M1  // 4 ==> Direction
const byte in2MotorRight = 51;   //28 C2 M2  // 3 == Dummy

// motor B (left)
const byte motorLeftEncoderPinA = 34; //26
const byte motorLeftEncoderPinB = 38; // 30
const byte enMotorLeft = 11; // 7 ==> speed
const byte in1MotorLeft = 13;    //30  // 6 ==> Direction
const byte in2MotorLeft = 51;    //32  // 5 ==> Dummy

#endif // ASTRO_CONFIG_H_
