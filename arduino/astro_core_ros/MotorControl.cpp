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
    ros::NodeHandle& aNh,
    MotorInstance aInstance,
    mc::Encoder& aEncoderObj,
    mc::DCMotor& aMotor
    ): Encoder( aInstance, aEncoderObj )
    , mMotor( aMotor )
    , mEncoder( aEncoderObj )
    , mInstance( aInstance )
    , mNh( aNh )
{
}


void MotorControl::InitMotorControl()
{
  String motormessage =  "InitMotorControl for motor " + String(mInstance);
  mNh.logwarn( motormessage.c_str() );
  mMotor.setDuty(0);
}

void MotorControl::SetMotorRateAndDirection
    (
    int aPwmRef,// Controls the speed
    const float aRateRef // Controls the direction
    )
{
      // avoid noisy pwm range
      if (abs(aRateRef) < 0.1)
        aPwmRef = 0;
    
      int duty = 50;
      duty = duty * aRateRef;
      if(duty > 100)
      {
        duty = 100;
      }
      if( duty < -100)
      {
        duty = -100;
      }
      String message =  "Setting aRateRef = " + String(aRateRef) + " aPwmRef = " + String(aPwmRef) + " Duty =" + String(duty) + " for motor " + String(mMotor.getInstanceId()) + " encoder = " + String(mEncoder.getInstance());
      mNh.loginfo( message.c_str() );
      mMotor.setDuty( duty );
}
