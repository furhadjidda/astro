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

#ifndef FOXBOT_CORE_CONFIG_H_
#define FOXBOT_CORE_CONFIG_H_

// enable this line for Arduino Due
#define USE_USBCON

/* Include librairies */

#include <Arduino.h>

/* Global parameters */
#define FREQUENCY_RATE 						30 			// [ms] default 50ms
#define FREQUENCY_ODOMETRY 				    150 		// [ms] default 250ms
#define FREQUENCY_ROSPINONCE 				150 		// [ms]
#define FREQUENCY_CONTROLLER 				30 			// [ms] default 50ms

/* Rate computing parameters */
#define RATE_DIRECTION_MEDIAN_FILTER_SIZE 	3
#define RATE_CONV 							0.006982 // 0.0073882 	// [inc] -> [rad]
// 853 (234) inc per wheel rotation
// RATE_CONV = 2*pi/(Nb_increments_per_wheel_rotation) = 2*pi/299
// In my case this is close to 299 which is why the value 0.021
#define RATE_AVERAGE_FILTER_SIZE 			4

/* Rate controller parameters */
#define PWM_MAX				 				4096            // 12 bit
#define RATE_CONTROLLER_KP 					150.0 		    // default 150
#define RATE_CONTROLLER_KD 					4500000000000.0 //4500000000000
#define RATE_CONTROLLER_KI 					0.00001 	    //0.00001
#define RATE_INTEGRAL_FREEZE				250
#define RATE_CONTROLLER_MIN_PWM 			-500
#define RATE_CONTROLLER_MAX_PWM 			500

/* Mechanical parameters */
#define WHEEL_RADIUS 						0.0325 		// [m]
// distance between the two wheels
#define BASE_LENGTH 						0.140 		// [m]  0.288

#endif // FOXBOT_CORE_CONFIG_H_
