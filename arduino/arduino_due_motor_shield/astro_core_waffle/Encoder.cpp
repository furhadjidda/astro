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

#include "Encoder.hpp"

Encoder *pointerToClass[MC_MAX]; // declare a pointer to MotorControl class

static void LeftEncoderInterruptHandler(void)  // define global handler
{
    pointerToClass[MC_LEFT]->IsrLeftCounterDirection(); // calls class member handler
}

static void RightEncoderInterruptHandler(void)  // define global handler
{
    pointerToClass[MC_RIGHT]->IsrRightCounterDirection(); // calls class member handler
}

void (*EncoderISR[MC_MAX]) (void);

Encoder::Encoder
    (
    byte aEncoderPinA,
    byte aEncoderPinB,    
    MotorInstance aInstance
    )
    : mEncoderPinA( aEncoderPinA )
    , mEncoderPinB( aEncoderPinB )
{
    pointerToClass[aInstance] = this;
    EncoderISR[MC_LEFT] = LeftEncoderInterruptHandler;
    EncoderISR[MC_RIGHT] = RightEncoderInterruptHandler;
    pinMode(mEncoderPinA, INPUT);
    attachInterrupt
        (
        digitalPinToInterrupt(mEncoderPinA),
        EncoderISR[aInstance],
        RISING
        );
    pinMode(mEncoderPinB, INPUT);
}

void Encoder::IsrLeftCounterDirection()
{
    mMotorInc ++;
    if ( mMotorCheckDirection == 1) 
    {
        if ( digitalRead(mEncoderPinB) && digitalRead(mEncoderPinA))
        {
            mMotorDirection = 1;
        } 
        else 
        {
            mMotorDirection = -1;
        }
        mMotorCheckDirection = 0;
    }
}

void Encoder::IsrRightCounterDirection()
{
    mMotorInc ++;
    if ( mMotorCheckDirection == 1) 
    {
        if ( digitalRead(mEncoderPinB) && digitalRead(mEncoderPinA))
        {
            mMotorDirection = 1;
        } 
        else 
        {
            mMotorDirection = -1;
        }
        mMotorCheckDirection = 1;
    }
}
