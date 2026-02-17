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

#ifndef STATUS_CLASS_HPP_
#define STATUS_CLASS_HPP_

#include <std_msgs/msg/int32.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#define RGB(_r, _g, _b) {.r = (_r), .g = (_g), .b = (_b)}

const struct led_rgb colors[] = {
    RGB(0xFF, 0x00, 0x00), /* white */
    RGB(0x00, 0xFF, 0x00), /* green */
    RGB(0x00, 0x00, 0xFF), /* teal */
};

class StatusClass {
   public:
    StatusClass();

    void display_color(const struct led_rgb& color);

#if DT_NODE_HAS_PROP(DT_ALIAS(led_strip), chain_length)
    static constexpr size_t num_pixels = DT_PROP(DT_ALIAS(led_strip), chain_length);
#else
#error "Unable to determine length of LED strip"
#endif
    void power_on(void);

    static const device* const strip;
    static const gpio_dt_spec power_ctrl;
    static inline led_rgb pixels[num_pixels];
};

#endif  // STATUS_CLASS_HPP_