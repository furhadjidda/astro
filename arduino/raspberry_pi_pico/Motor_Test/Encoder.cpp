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
#include "Common.hpp"
#include <sys/_stdint.h>

Encoder* pointerToClass[MotorId::MaxMotorValue]; // declare a pointer to MotorControl class
// problem with this is that if multiple instances of the MotorControl class gets created this gets overwritten and there is also a memory leak.

static void LeftEncoderInterruptHandler(void) // define global handler
{
    pointerToClass[MotorId::MotorB]->IsrLeftCounterDirection(); // calls class member handler
}

static void RightEncoderInterruptHandler(void) // define global handler
{
    pointerToClass[MotorId::MotorA]->IsrRightCounterDirection(); // calls class member handler
}

void (*EncoderISR[MotorId::MaxMotorValue])(void);

Encoder::Encoder()
{
}

void Encoder::SetEncoderPins(MotorId aMotor, uint8_t aEncoderPinA, uint8_t aEncoderPinB)
{
    mEncoderPinA = aEncoderPinA;
    mEncoderPinB = aEncoderPinB;
    pointerToClass[aMotor] = this;
    EncoderISR[MotorId::MotorB] = LeftEncoderInterruptHandler;
    EncoderISR[MotorId::MotorA] = RightEncoderInterruptHandler;
    pinMode(mEncoderPinA, INPUT);
    attachInterrupt(
        digitalPinToInterrupt(mEncoderPinA),
        EncoderISR[aMotor],
        RISING);
    pinMode(mEncoderPinB, INPUT);
}

int Encoder::GetIncermentValue()
{
    return mMotorInc;
}

int Encoder::GetResolution()
{
    return 0;
}

int Encoder::GetDirection()
{
    return mDirection;
}

int Encoder::GetVelocity()
{
    return 0;
}

void Encoder::SetResolution(int aResolution)
{
}

void Encoder::SetIncrementValue(int aValue)
{
    mMotorInc = 0;
}

void Encoder::IsrLeftCounterDirection()
{
    mMotorInc++;
    if (digitalRead(mEncoderPinB) && digitalRead(mEncoderPinA)) {
        mDirection = 1;
    } else {
        mDirection = -1;
    }
}

void Encoder::IsrRightCounterDirection()
{
    mMotorInc++;
    if (digitalRead(mEncoderPinB) && digitalRead(mEncoderPinA)) {
        mDirection = 1;
    } else {
        mDirection = -1;
    }
}
