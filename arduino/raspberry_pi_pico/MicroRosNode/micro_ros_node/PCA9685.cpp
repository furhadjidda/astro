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
#include "PCA9685.hpp"
#include "Wire.h"
#include <math.h>
#include <sys/_stdint.h>

static const uint8_t __SUBADR1 = 0x02;
static const uint8_t __SUBADR2 = 0x03;
static const uint8_t __SUBADR3 = 0x04;
static const uint8_t __MODE1 = 0x00;
static const uint8_t __PRESCALE = 0xFE;
static const uint8_t __LED0_ON_L = 0x06;
static const uint8_t __LED0_ON_H = 0x07;
static const uint8_t __LED0_OFF_L = 0x08;
static const uint8_t __LED0_OFF_H = 0x09;
static const uint8_t __ALLLED_ON_L = 0xFA;
static const uint8_t __ALLLED_ON_H = 0xFB;
static const uint8_t __ALLLED_OFF_L = 0xFC;
static const uint8_t __ALLLED_OFF_H = 0xFD;

static const int SDIO = 20;
static const int SCLK = 21;
static const int freq = 100000;
static const uint8_t address = 0x40;

// This is needed as by default the I2c pin numbers are default and does not match what i have connected to
arduino::MbedI2C WireI2c(SDIO, SCLK);

PCA9685::PCA9685()
{
}

void PCA9685::PwmInit()
{
    // This initialized the I2C
    //Wire.setSDA(SDIO);
    pinMode(SDIO, OUTPUT);
    //Wire.setSCL(SCLK);
    WireI2c.begin();
    //Reseting PCA9685
    delay(5000);
    I2cWrite(__MODE1, 0x06);
}

void PCA9685::SetPWMFreq(unsigned aFrequency)
{
    // Sets the PWM frequency
    uint8_t oldmode = 0x00;
    unsigned prescaleval = 25000000.0; // 25MHz
    prescaleval /= 4096.0; // 12-bit
    prescaleval /= aFrequency;
    prescaleval -= 1.0;
    Serial.println("Setting PWM frequency to " + String(aFrequency) + "Hz");
    Serial.println("Estimated pre-scale " + String(prescaleval));

    unsigned prescale = floor(prescaleval + 0.5);
    Serial.println("Final pre-scale " + String(prescale));

    // Read I2C
    WireI2c.beginTransmission(address);
    WireI2c.write(__MODE1);
    WireI2c.endTransmission();
    int bytesRead = WireI2c.requestFrom(address, 1);
    if (bytesRead == 1) {
        oldmode = WireI2c.read();
    }

    Serial.println("oldmode = " + String(oldmode));
    uint8_t newmode = (oldmode & 0x7F) | 0x10; // sleep
    I2cWrite(__MODE1, newmode);

    I2cWrite(__PRESCALE, prescale);

    I2cWrite(__MODE1, oldmode);

    delay(500);

    I2cWrite(__MODE1, oldmode | 0x80);
}

void PCA9685::SetPWM(uint16_t aChannel, uint16_t aOn, uint16_t aOff)
{
    // Sets a single PWM channel
    I2cWrite(__LED0_ON_L + 4 * aChannel, aOn & 0xFF);
    I2cWrite(__LED0_ON_H + 4 * aChannel, aOn >> 8);

    I2cWrite(__LED0_OFF_L + 4 * aChannel, aOff & 0xFF);
    I2cWrite(__LED0_OFF_H + 4 * aChannel, aOff >> 8);
    Serial.println("Channel " + String(aChannel) + " LED_ON " + String(aOn) + " LED_OFF " + String(aOff));
}

void PCA9685::SetServoPulse(uint16_t aChannel, uint16_t aPulse)
{
    //unsigned int pulse = aPulse * (4095 / 100);
    unsigned int pulse = aPulse;
    SetPWM(aChannel, 0, pulse);
}

void PCA9685::SetLevel(uint16_t aChannel, uint16_t aValue)
{
    if (aValue == 1) {
        SetPWM(aChannel, 0, 4095);
    } else {
        SetPWM(aChannel, 0, 0);
    }
}

void PCA9685::I2cWrite(uint8_t aCommand, uint8_t aValue)
{
    WireI2c.beginTransmission(address);
    WireI2c.write(aCommand);
    WireI2c.write(aValue);
    WireI2c.endTransmission();
}
