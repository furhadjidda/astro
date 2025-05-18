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

#include "motor_control.hpp"
#include "pico/stdlib.h"
#include <cmath>
#include <sys/_intsup.h>
#include <sys/_stdint.h>
#include <sys/_types.h>

static const uint8_t directionFwd[2] = {1, 0};
static const uint8_t directionBkwd[2] = {0, 1};
static const uint8_t *direction[MotorDirectionMaxValue] = {directionFwd, directionBkwd};

motor_control::motor_control() {}

void motor_control::Init() {
    mMotor[MotorId::MotorA].SetMotorPins(0, 1, 2);
    mMotor[MotorId::MotorB].SetMotorPins(3, 4, 5);
    mEncoder[MotorId::MotorA].SetEncoderPins(MotorId::MotorA, 14, 15);
    mEncoder[MotorId::MotorB].SetEncoderPins(MotorId::MotorB, 16, 17);
    mEncoder[MotorId::MotorA].SetIncrementValue(0);
    mEncoder[MotorId::MotorB].SetIncrementValue(0);
    mPwm.PwmInit();
    mPwm.SetPwmFrequency(50);
    printf("motor init complete\n");
}

void motor_control::MotorRun(MotorId aMotorId, MotorDirection aDirection, uint8_t aSpeed) {
    if (aSpeed > 4096)
        return;

    // Serial.println("set PWM PIN " + std::string(mMotor[aMotorId].GetPwmPinValue()) + " speed = " +
    // std::string(aSpeed)); Serial.println("set pin A " + std::string(mMotor[aMotorId].GetPinAValue()) +
    //                " direction = " + std::string(direction[aDirection][0]));
    // Serial.println("set pin B " + std::string(mMotor[aMotorId].GetPinBValue()) +
    //                " direction = " + std::string(direction[aDirection][1]));

    mPwm.SetServoPulse(mMotor[aMotorId].GetPwmPinValue(), aSpeed);
    mPwm.SetLevel(mMotor[aMotorId].GetPinAValue(), direction[aDirection][0]);
    mPwm.SetLevel(mMotor[aMotorId].GetPinBValue(), direction[aDirection][1]);
}

void motor_control::SetSpeed(MotorId aMotorId, int8_t aSpeed) {
    mPwm.SetServoPulse(mMotor[aMotorId].GetPwmPinValue(), aSpeed);
    if (aSpeed < 0) {
        mPwm.SetLevel(mMotor[aMotorId].GetPinAValue(), 0);
        mPwm.SetLevel(mMotor[aMotorId].GetPinBValue(), 1);
    } else {
        mPwm.SetLevel(mMotor[aMotorId].GetPinAValue(), 1);
        mPwm.SetLevel(mMotor[aMotorId].GetPinBValue(), 0);
    }
}

void motor_control::SetMotorRateAndDirection(MotorId aMotorId, int aPwmRef, const float aRateRef) {
    // avoid noisy pwm range
    if (abs(aRateRef) < 0.1)
        aPwmRef = 0;

    // write pwm -- CONTROLS THE SPEED
    mPwm.SetServoPulse(mMotor[aMotorId].GetPwmPinValue(), abs(aPwmRef));

    // write direction
    if (aRateRef > 0) {
        // mPwm.SetLevel(mMotor[aMotorId].GetPwmPinValue(), 1);
        mPwm.SetLevel(mMotor[aMotorId].GetPinAValue(), 1);
        mPwm.SetLevel(mMotor[aMotorId].GetPinBValue(), 0);
    } else if (aRateRef < 0) {
        // mPwm.SetLevel(mMotor[aMotorId].GetPwmPinValue(), 1);
        mPwm.SetLevel(mMotor[aMotorId].GetPinAValue(), 0);
        mPwm.SetLevel(mMotor[aMotorId].GetPinBValue(), 1);
    }
}

int motor_control::GetEncoderIncrementValue(MotorId aMotorId) { return mEncoder[aMotorId].GetIncrementValue(); }

void motor_control::ResetEncoderIncrementValue(MotorId aMotorId) { mEncoder[aMotorId].SetIncrementValue(0); }

void motor_control::MotorStop() {
    mPwm.SetServoPulse(mMotor[MotorId::MotorA].GetPwmPinValue(), 0);
    mPwm.SetServoPulse(mMotor[MotorId::MotorB].GetPwmPinValue(), 0);
}

uint64_t motor_control::DistanceToTicks(uint32_t aDistance) {
    // 1 Revolution = ppr , which is equal to 2*pi*wheelRadius( distance)
    // so 2*pi*wheelRadius is the distance the robot travels in 1 revolution
    double multiplier = aDistance / (2 * M_PI * wheelRadius);
    // Serial.println("distance =" + std::string(aDistance) + " multiplier = " + std::string(multiplier));
    // Serial.println("DistanceToTicks =" + std::string(ppr * multiplier));
    uint64_t ticks = ceil(ppr * multiplier);
    return ticks;
}

void motor_control::AngularVelocity(MotorId aId) {
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
        // Serial.println(" currentMillis = " + currentMillis );
        if (currentMillis - previousMillis > interval) {

            previousMillis = currentMillis;
            CalculateAngularVelocity(aId, AngularVelARad, AngularVelADegree, RpmA);
            // Serial.println(" Speed: " + std::string(RpmA) + " RPM");
            // Serial.println(" Angular Velocity: " + std::string(AngularVelARad) + " radians/sec  OR " +
            //                std::string(AngularVelADegree) + " Degree/Second");
            mEncoder[aId].SetIncrementValue(0);
            --loopVal;
        }
    }
    MotorStop();
}

void motor_control::CalculateAngularVelocity(MotorId aMotorId, float &aAngluarRad, float &aAngularDeg, float &aRpm) {
    aRpm = (float)(mEncoder[aMotorId].GetIncrementValue() * 60 / ppr);
    aAngluarRad = aRpm * RpmToRadians;
    aAngularDeg = aAngluarRad * RadiansToDegree;
}

int motor_control::GetMotorDirection(MotorId aMotorId) { return mEncoder[aMotorId].GetDirection(); }
