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
#ifndef DISPLAY_HPP
#define DISPLAY_HPP
// GNSS Sensor
#include <memory>

#include "SSD1306Oled.hpp"
#include "display_driver.hpp"
#include "font7x5.hpp"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "sensor.hpp"
class DisplaySensor : public Sensor {
   public:
    bool initialize(i2c_inst_t *aI2cInstance) override;
    virtual void get_debug_message(std_msgs__msg__String &msg) override;
    void create_welcome_screen();

   private:
    std::unique_ptr<SSD1306Oled> display;  // = std::make_unique<SSD1306Oled>(i2c0, 4, 5, 0x3c);
};

#endif  // DISPLAY_HPP