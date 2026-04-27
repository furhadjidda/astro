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

#include "encoder.hpp"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <cstdint>

// Static member definition
Encoder *Encoder::pointerToClass[static_cast<int>(MotorId::MaxMotorValue)] = {nullptr, nullptr};

// Constructor
Encoder::Encoder() {}

// Set encoder pins and attach interrupts
void Encoder::SetEncoderPins(MotorId motor, uint8_t encoderPinA, uint8_t encoderPinB) {
    mEncoderPinA = encoderPinA;
    mEncoderPinB = encoderPinB;

    // Store pointer to this instance
    pointerToClass[static_cast<int>(motor)] = this;

    // Configure pins as input
    gpio_init(mEncoderPinA);
    gpio_set_dir(mEncoderPinA, GPIO_IN);
    gpio_pull_up(mEncoderPinA);

    gpio_init(mEncoderPinB);
    gpio_set_dir(mEncoderPinB, GPIO_IN);
    gpio_pull_up(mEncoderPinB);

    // Enable interrupts
    gpio_set_irq_enabled_with_callback(mEncoderPinA, GPIO_IRQ_EDGE_RISE, true, GlobalInterruptHandler);
}

// Get increment value
int Encoder::GetIncrementValue() { return mMotorInc; }

// Get resolution (not implemented in this example)
int Encoder::GetResolution() { return 0; }

// Get direction
int Encoder::GetDirection() { return mDirection; }

// Get velocity (not implemented in this example)
int Encoder::GetVelocity() { return 0; }

// Set resolution (not implemented in this example)
void Encoder::SetResolution(int resolution) {}

// Reset increment value
void Encoder::SetIncrementValue(int value) { mMotorInc = 0; }

// Handle left counter direction
void Encoder::IsrLeftCounterDirection() {
    mMotorInc++;
    if (gpio_get(mEncoderPinB) && gpio_get(mEncoderPinA)) {
        mDirection = 1;
    } else {
        mDirection = -1;
    }
}

// Handle right counter direction
void Encoder::IsrRightCounterDirection() {
    mMotorInc++;
    if (gpio_get(mEncoderPinB) && gpio_get(mEncoderPinA)) {
        mDirection = 1;
    } else {
        mDirection = -1;
    }
}

// Global interrupt handler
void Encoder::GlobalInterruptHandler(uint gpio, uint32_t events) {
    for (int i = 0; i < static_cast<int>(MotorId::MaxMotorValue); ++i) {
        if (pointerToClass[i] != nullptr && pointerToClass[i]->mEncoderPinA == gpio) {
            if (i == static_cast<int>(MotorId::MotorB)) {
                pointerToClass[i]->IsrLeftCounterDirection();
            } else if (i == static_cast<int>(MotorId::MotorA)) {
                pointerToClass[i]->IsrRightCounterDirection();
            }
        }
    }
}
