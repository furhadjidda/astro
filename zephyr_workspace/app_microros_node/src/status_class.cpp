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

#include "status_class.hpp"

const gpio_dt_spec StatusClass::power_ctrl = GPIO_DT_SPEC_GET(DT_NODELABEL(neopixel_pwr), enable_gpios);

const device* const StatusClass::strip = DEVICE_DT_GET(DT_ALIAS(led_strip));

StatusClass::StatusClass() { power_on(); }

void StatusClass::display_color(const struct led_rgb& color) {
    if (!device_is_ready(strip)) {
        return;
    }

    memset(&pixels, 0x00, sizeof(pixels));
    memcpy(&pixels[0], &color, sizeof(struct led_rgb));
    led_strip_update_rgb(strip, pixels, num_pixels);
}

void StatusClass::power_on(void) {
    if (!device_is_ready(power_ctrl.port)) {
        return;
    }

    gpio_pin_configure_dt(&power_ctrl, GPIO_OUTPUT_ACTIVE);
}