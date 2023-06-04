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

#include "MotorControl.hpp"

MotorControl::MotorControl
    (    
    const byte aEnMotor, // This controls the speed
    const byte aIn1Motor,// This controls the direction of motor
    const byte aIn2Motor, // Not used with Arduino motor shield
    byte aEncoderPinA,
    byte aEncoderPinB,
    MotorInstance aInstance
    )
    : Encoder( aEncoderPinA, aEncoderPinB, aInstance  )
{
    mEnMotor = aEnMotor;
    mIn1Motor = aIn1Motor;
    mIn2Motor = aIn2Motor;
}


void MotorControl::InitMotorControl()
{  
    pinMode(mEnMotor, OUTPUT);
    pinMode(mIn1Motor, OUTPUT);
    pinMode(mIn2Motor, OUTPUT);
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
    
      // write direction
      if (aRateRef > 0) 
      {
        digitalWrite(mIn1Motor, LOW);
        //digitalWrite(mIn2Motor, HIGH);// Not used with Arduino motor shield
      }
      else if (aRateRef < 0) 
      {
        digitalWrite(mIn1Motor, HIGH);
        //digitalWrite(mIn2Motor, LOW);// Not used with Arduino motor shield
      }
    
      // write pwm -- CONTROLS THE SPEED
      analogWrite(mEnMotor, aPwmRef);
}
