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
#include <cmath>
#include <sys/_intsup.h>
#include <sys/_stdint.h>
#include <sys/_types.h>

static const uint8_t directionFwd[2] = { 1, 0 };
static const uint8_t directionBkwd[2] = { 0, 1 };
static const uint8_t* direction[MotorDirectionMaxValue] = { directionFwd, directionBkwd };

MotorControl::MotorControl()
{
}

void MotorControl::Init()
{
    mMotor[MotorId::MotorA].SetMotorPins(0, 1, 2);
    mMotor[MotorId::MotorB].SetMotorPins(3, 4, 5);
    mEncoder[MotorId::MotorA].SetEncoderPins(MotorId::MotorA, 14, 15);
    mEncoder[MotorId::MotorB].SetEncoderPins(MotorId::MotorB, 16, 17);
    mEncoder[MotorId::MotorA].SetIncrementValue(0);
    mEncoder[MotorId::MotorB].SetIncrementValue(0);
    mPwm.PwmInit();
    mPwm.SetPWMFreq(50);
}

void MotorControl::MotorRun(MotorId aMotorId, MotorDirection aDirection, uint8_t aSpeed)
{
    if (aSpeed > 4096)
        return;

    Serial.println("set PWM PIN " + String(mMotor[aMotorId].GetPwmPinValue()) + " speed = " + String(aSpeed));
    Serial.println("set pin A " + String(mMotor[aMotorId].GetPinAValue()) + " direction = " + String(direction[aDirection][0]));
    Serial.println("set pin B " + String(mMotor[aMotorId].GetPinBValue()) + " direction = " + String(direction[aDirection][1]));

    mPwm.SetServoPulse(mMotor[aMotorId].GetPwmPinValue(), aSpeed);
    mPwm.SetLevel(mMotor[aMotorId].GetPinAValue(), direction[aDirection][0]);
    mPwm.SetLevel(mMotor[aMotorId].GetPinBValue(), direction[aDirection][1]);
}

void MotorControl::Forward(uint8_t aSpeed, uint32_t aDistance)
{
    // Direction: FWD =  PINA 1 , PINB 0

    mPwm.SetServoPulse(mMotor[MotorId::MotorA].GetPwmPinValue(), aSpeed);
    mPwm.SetLevel(mMotor[MotorId::MotorA].GetPinAValue(), 1);
    mPwm.SetLevel(mMotor[MotorId::MotorA].GetPinBValue(), 0);

    mPwm.SetServoPulse(mMotor[MotorId::MotorB].GetPwmPinValue(), aSpeed);
    mPwm.SetLevel(mMotor[MotorId::MotorB].GetPinAValue(), 1);
    mPwm.SetLevel(mMotor[MotorId::MotorB].GetPinBValue(), 0);

    int encoderTick = 0;
    int targetTickValue = DistanceToTicks(aDistance);
    while (encoderTick <= targetTickValue) {
        Serial.println("Encoder Tick = " + String(encoderTick));
        int encodeTickA = mEncoder[MotorId::MotorA].GetIncreamentValue();
        int encodeTickB = mEncoder[MotorId::MotorB].GetIncreamentValue();
        encoderTick = (encodeTickA + encodeTickB) / 2;
    }
    mEncoder[MotorId::MotorA].SetIncrementValue(0);
    mEncoder[MotorId::MotorB].SetIncrementValue(0);
    MotorStop();
}

void MotorControl::Backward(uint8_t aSpeed)
{
    mPwm.SetServoPulse(mMotor[MotorId::MotorA].GetPwmPinValue(), aSpeed);
    mPwm.SetLevel(mMotor[MotorId::MotorA].GetPinAValue(), 0);
    mPwm.SetLevel(mMotor[MotorId::MotorA].GetPinBValue(), 1);

    mPwm.SetServoPulse(mMotor[MotorId::MotorB].GetPwmPinValue(), aSpeed);
    mPwm.SetLevel(mMotor[MotorId::MotorB].GetPinAValue(), 0);
    mPwm.SetLevel(mMotor[MotorId::MotorB].GetPinBValue(), 1);
}

void MotorControl::Right(uint8_t aSpeed)
{
    mPwm.SetServoPulse(mMotor[MotorId::MotorA].GetPwmPinValue(), aSpeed);
    mPwm.SetLevel(mMotor[MotorId::MotorA].GetPinAValue(), 0);
    mPwm.SetLevel(mMotor[MotorId::MotorA].GetPinBValue(), 1);

    mPwm.SetServoPulse(mMotor[MotorId::MotorB].GetPwmPinValue(), aSpeed);
    mPwm.SetLevel(mMotor[MotorId::MotorB].GetPinAValue(), 1);
    mPwm.SetLevel(mMotor[MotorId::MotorB].GetPinBValue(), 0);
}

void MotorControl::Left(uint8_t aSpeed)
{
    mPwm.SetServoPulse(mMotor[MotorId::MotorA].GetPwmPinValue(), aSpeed);
    mPwm.SetLevel(mMotor[MotorId::MotorA].GetPinAValue(), 1);
    mPwm.SetLevel(mMotor[MotorId::MotorA].GetPinBValue(), 0);

    mPwm.SetServoPulse(mMotor[MotorId::MotorB].GetPwmPinValue(), aSpeed);
    mPwm.SetLevel(mMotor[MotorId::MotorB].GetPinAValue(), 0);
    mPwm.SetLevel(mMotor[MotorId::MotorB].GetPinBValue(), 1);
}

void MotorControl::SetSpeed(MotorId aMotorId, int8_t aSpeed)
{
    mPwm.SetServoPulse(mMotor[aMotorId].GetPwmPinValue(), aSpeed);
    if( aSpeed < 0)
    {
        mPwm.SetLevel(mMotor[aMotorId].GetPinAValue(), 0);
        mPwm.SetLevel(mMotor[aMotorId].GetPinBValue(), 1);
    }
    else {
        mPwm.SetLevel(mMotor[aMotorId].GetPinAValue(), 1);
        mPwm.SetLevel(mMotor[aMotorId].GetPinBValue(), 0);
    }
}

void MotorControl::SetMotorRateAndDirection(MotorId aMotorId, int aPwmRef, const float aRateRef)
{
    // avoid noisy pwm range
    if (abs(aRateRef) < 0.1)
      aPwmRef = 0;

    // write direction
    if (aRateRef > 0)
    {
        mPwm.SetLevel(mMotor[aMotorId].GetPinAValue(), 1);
        mPwm.SetLevel(mMotor[aMotorId].GetPinBValue(), 0);
    }
    else if (aRateRef < 0)
    {
        mPwm.SetLevel(mMotor[aMotorId].GetPinAValue(), 0);
        mPwm.SetLevel(mMotor[aMotorId].GetPinBValue(), 1);
    }
    // write pwm -- CONTROLS THE SPEED
    mPwm.SetServoPulse(mMotor[aMotorId].GetPwmPinValue(), abs(aPwmRef));
}



int MotorControl::GetEncoderIncrementValue(MotorId aMotorId)
{
    return mEncoder[aMotorId].GetIncreamentValue();
}

void MotorControl::ResetEncoderIncrementValue(MotorId aMotorId)
{
    mEncoder[aMotorId].SetIncrementValue(0);
}

void MotorControl::MotorStop()
{
    mPwm.SetServoPulse(mMotor[MotorId::MotorA].GetPwmPinValue(), 0);
    mPwm.SetServoPulse(mMotor[MotorId::MotorB].GetPwmPinValue(), 0);
}

uint64_t MotorControl::DistanceToTicks(uint32_t aDistance)
{
    // 1 Revolution = ppr , which is equal to 2*pi*wheelRadius( distance)
    // so 2*pi*wheelRadius is the distance the robot travels in 1 revolution
    double multiplier = aDistance / (2 * M_PI * wheelRadius);
    Serial.println("distance =" + String(aDistance) + " multiplier = " + String(multiplier));
    Serial.println("DistanceToTicks =" + String(ppr * multiplier));
    uint64_t ticks = ceil(ppr * multiplier);
    return ticks;
}

void MotorControl::AngularVelocity(MotorId aId)
{
    int loopVal = 10;
    long previousMillis = 0;
    long currentMillis = 0;
    float AngularVelARad = 0;
    float AngularVelADegree = 0;
    float RpmA = 0;
    int interval = 1000;

    MotorRun(aId, MotorDirection::Forward, 100);
    while (loopVal > 0) {
        currentMillis = millis();
        //Serial.println(" currentMillis = " + currentMillis );
        if (currentMillis - previousMillis > interval) {

            previousMillis = currentMillis;
            CalculateAngularVelocity(aId, AngularVelARad, AngularVelADegree, RpmA);
            Serial.println(" Speed: " + String(RpmA) + " RPM");
            Serial.println(" Angular Velocity: " + String(AngularVelARad) + " radians/sec  OR " + String(AngularVelADegree) + " Degree/Second");
            mEncoder[aId].SetIncrementValue(0);
            --loopVal;
        }
    }
    MotorStop();
}

void MotorControl::CalculateAngularVelocity(MotorId aMotorId, float& aAngluarRad, float& aAngularDeg, float& aRpm)
{
    aRpm = (float)(mEncoder[aMotorId].GetIncreamentValue() * 60 / ppr);
    aAngluarRad = aRpm * RpmToRadians;
    aAngularDeg = aAngluarRad * RadiansToDegree;
}

int MotorControl::GetMotorDirection(MotorId aMotorId)
{
    return mEncoder[aMotorId].GetDirection();
}
