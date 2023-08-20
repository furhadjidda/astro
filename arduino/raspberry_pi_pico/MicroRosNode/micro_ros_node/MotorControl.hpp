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

#include <sys/_stdint.h>
#ifndef __MOTORCONTROL_HPP__
#define __MOTORCONTROL_HPP__

#include "Common.hpp"
#include "Encoder.hpp"
#include "Motor.hpp"
#include "PCA9685.hpp"

class MotorControl {

public:
    MotorControl();
    void Init();
    void MotorRun(MotorId aMotorId, MotorDirection aDirection, uint8_t aSpeed);
    void MotorStop();
    void Forward(uint8_t aSpeed, uint32_t aDistance); // Distance is in mm
    void Backward(uint8_t aSpeed);
    void Right(uint8_t aSpeed);
    void Left(uint8_t aSpeed);
    void SetSpeed(MotorId aMotorId, int8_t aSpeed);
    void SetMotorRateAndDirection(MotorId aMotorId, int aPwmRef, const float aRateRef);
    int GetEncoderIncrementValue(MotorId aMotorId);
    void ResetEncoderIncrementValue(MotorId aMotorId);
    void AngularVelocity(MotorId aId);

    int CalculateAngularPosition(double aLeftWheelVelocity, double aRightWheelVelocity);
    int GetMotorDirection(MotorId aMotorId);

private:
    PCA9685 mPwm;
    Motor mMotor[MotorId::MaxMotorValue];
    Encoder mEncoder[MotorId::MaxMotorValue];
    uint64_t DistanceToTicks(uint32_t aDistance);
    void CalculateAngularVelocity(MotorId aMotorId, float& aAngluarRad, float& aAngularDeg, float& aRpm);
};

#endif