#include "api/Common.h"
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

#include <stdint.h>

#include "MotorControl.hpp"

MotorControl::MotorControl
    (
    ros::NodeHandle &aNh,
    MotorInstance aInstance,
    mc::Encoder &aEncoderObj,
    mc::DCMotor &aMotor
    ) 
    : Encoder(aInstance, aEncoderObj)
    , mMotor(aMotor)
    , mEncoder(aEncoderObj)
    , mInstance(aInstance)
    , mNh(aNh)
{
}

void MotorControl::InitMotorControl()
{
    String motormessage = "InitMotorControl for motor " + String(mInstance);
    mNh.logwarn(motormessage.c_str());
    mMotor.setDuty(0);
}

void MotorControl::SetDutyCycle
    (
    int aVal // Controls the speed
    )
{
    int duty = 50;
    duty = aVal;
    duty = constrain(duty, -95, 95);
    String message = "Setting Duty Cycle = " + String(duty) + " Original Duty =" + String(aVal) + " for motor " + String(mMotor.getInstanceId()) + " encoder = " + String(mEncoder.getInstance());
    mNh.loginfo(message.c_str());
    mMotor.setDuty(duty);
}
