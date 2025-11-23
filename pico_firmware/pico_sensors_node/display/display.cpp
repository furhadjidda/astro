/*
 *   This file is part of astro.
 *
 *   astro is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   astro is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License v3.0
 *   along with astro.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "display.hpp"

#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

bool DisplaySensor::initialize(i2c_inst_t *aI2cInstance) {
    display = std::make_unique<SSD1306Oled>(aI2cInstance, 4, 5, 0x3c);
    if (!display) {
        return false;  // Handle allocation failure
    }

    display->Begin();             // Corrected method name
    display->SetFont(&f7x5font);  // Ensure correct method name
    return true;
}

void DisplaySensor::get_debug_message(std_msgs__msg__String &msg) {}

void DisplaySensor::create_welcome_screen() {
    display->DrawString(0, 0 - display->GetFontHeight(display->GetFont()), "    Welcome to Astro   ", 1);
    display->DrawHLine(5, 10, 118, 1);
    display->DrawHLine(5, 13, 118, 1);

    display->DrawString(0, 30, "All Systems Initialized !!", 1);
    display->DrawString(0, 20, "Micro-ROS", 1);
    display->DrawString(0, 10, "Sensor Node", 1);
    display->Display();
}