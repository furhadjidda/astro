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

#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <Arduino.h>
#include "Encoder.hpp"

class MotorControl: public Encoder{
    public:
        MotorControl
            (    
            const byte aEnMotor, // This controls the speed
            const byte aIn1Motor,// This controls the direction of motor
            const byte aIn2Motor, // Not used with Arduino motor shield
            byte aEncoderPinA,
            byte aEncoderPinB,
            MotorInstance aInstance
            );
            
        virtual ~MotorControl()
        {            
        }

        void InitMotorControl();
        
        void SetMotorRateAndDirection
            (
            int aPwmRef,
            const float aRateRef
            );

    private:
        byte mEnMotor;
        byte mIn1Motor;
        byte mIn2Motor;
};
#endif
