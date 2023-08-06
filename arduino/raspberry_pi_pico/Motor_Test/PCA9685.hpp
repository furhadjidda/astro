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
#ifndef __PCA9685_HPP__
#define __PCA9685_HPP__
#include "Arduino.h"
class PCA9685 {

public:
    PCA9685();
    void PwmInit();
    void SetPWMFreq(unsigned aFrequency);
    void SetPWM(uint16_t aChannel, uint16_t aOn, uint16_t aOff);
    void SetServoPulse(uint16_t aChannel, uint16_t aPulse);
    void SetLevel(uint16_t aChannel, uint16_t aValue);

private:
    void I2cWrite(uint8_t aCommand, uint8_t aValue);
};

#endif