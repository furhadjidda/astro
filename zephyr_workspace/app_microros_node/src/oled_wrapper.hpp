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

#ifndef OLED_WRAPPER_HPP
#define OLED_WRAPPER_HPP
#include <zephyr/device.h>
#include <zephyr/display/cfb.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

class OLEDWrapper {
   public:
    OLEDWrapper(const struct device* display_dev);

    int init();

    void print(const char* message, int x, int y);

    void clear();

    void finalize();

   private:
    const struct device* display_dev;
    uint16_t _rows = 0;
    uint8_t _ppt = 0;
    uint8_t _font_width = 0;
    uint8_t _font_height = 0;
};

#endif  // OLED_WRAPPER_HPP