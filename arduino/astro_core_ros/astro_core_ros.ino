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
#include "RosCommunication.hpp"

RosCommunication gComm;
/* Set the delay between fresh samples */
uint16_t DELAY_MS = 100;

void setup() 
{
    // put your setup code here, to run once:
    gComm.InitNode();
}

void loop() 
{
    // put your main code here, to run repeatedly:
    gComm.PublishData();
    gComm.SpinOnce();
    delay(DELAY_MS);
}
