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
 *   Description: Main file of the program.
 *   Author: Furhad Jidda
 *
 *   Serial communication is enabled with the following command:
 *   rosrun rosserial_python serial_node.py _port:=/dev/ttyACMx _baud:=115200
 */

#include "DriveTrain.hpp"
#include "AstroConfig.hpp"
uint16_t DELAY_MS = 1;
/* Define frequency loops */
DriveTrain gDriveTrain;

void setup()
{
    gDriveTrain.InitNode();
}

void loop() 
{
    gDriveTrain.UpdateOdometry();
    delay(DELAY_MS);
}
