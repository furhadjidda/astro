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

#ifndef __ENCODER_HPP__
#define __ENCODER_HPP__

#include "common.hpp"

class Encoder {
  public:
    Encoder();

    void SetEncoderPins(MotorId aMotor, uint8_t aEncoderPinA, uint8_t aEncoderPinB);

    int GetIncrementValue();
    int GetResolution();
    int GetDirection();
    int GetVelocity();

    void SetResolution(int aResolution);
    void SetIncrementValue(int aValue);

    void IsrLeftCounterDirection();
    void IsrRightCounterDirection();

  private:
    volatile int mMotorInc = 0;
    int mDirection;
    int mResolution{ppr};
    int mTicksPerRevolution;
    int mVelocity;
    uint8_t mEncoderPinA;
    uint8_t mEncoderPinB;
    static Encoder *pointerToClass[static_cast<int>(MotorId::MaxMotorValue)];
    static void GlobalInterruptHandler(uint gpio, uint32_t events);
};

#endif
