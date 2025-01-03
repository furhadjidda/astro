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

#ifndef __MOTOR_HPP__
#define __MOTOR_HPP__

#include <sys/_stdint.h>
#include <sys/types.h>

class Motor {

  public:
    void SetMotorPins(uint8_t aPwm, uint8_t aPinA, uint8_t aPinB) {
        mPwm = aPwm;
        mPinA = aPinA;
        mPinB = aPinB;
    }

    uint8_t GetPwmPinValue() { return mPwm; }

    uint8_t GetPinAValue() { return mPinA; }

    uint8_t GetPinBValue() { return mPinB; }

  private:
    uint8_t mPwm;
    uint8_t mPinA;
    uint8_t mPinB;
};

#endif