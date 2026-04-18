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

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/display/cfb.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ssd1306_sample, LOG_LEVEL_DBG);
#define MAX_FONTS 42

#define SELECTED_FONT_INDEX 0
static uint16_t rows = 0;
static uint8_t ppt = 0;
static uint8_t font_width = 0;
static uint8_t font_height = 0;

static const struct device* display_dev = DEVICE_DT_GET(DT_NODELABEL(ssd1306));

void main(void) {
    if (!device_is_ready(display_dev)) {
        printk("Display device not ready\n");
        return;
    }

    if (display_set_pixel_format(display_dev, PIXEL_FORMAT_MONO01) != 0) {
        printk("Failed to set required pixel format");
        return;
    }

    if (cfb_framebuffer_init(display_dev)) {
        printk("Framebuffer init failed\n");
        return;
    }

    cfb_framebuffer_clear(display_dev, true);

    display_blanking_off(display_dev);
    display_set_orientation(display_dev, DISPLAY_ORIENTATION_ROTATED_180);

    rows = cfb_get_display_parameter(display_dev, CFB_DISPLAY_ROWS);
    ppt = cfb_get_display_parameter(display_dev, CFB_DISPLAY_PPT);

    for (int idx = 0; idx < MAX_FONTS; idx++) {
        if (cfb_get_font_size(display_dev, idx, &font_width, &font_height)) {
            break;  // end of font list, so exit loop.
        }

        printk("index[%d] font width %d, font height %d", idx, font_width, font_height);
    }

    cfb_framebuffer_set_font(display_dev, SELECTED_FONT_INDEX);

    cfb_print(display_dev, "Hello Zephyr! Testing a long long message ", 0, 0);  // Print at x=0, y=0
    cfb_framebuffer_invert(display_dev);    // Optional: Invert the display (bright text on dark background)
    cfb_framebuffer_finalize(display_dev);  // Update the display
}
