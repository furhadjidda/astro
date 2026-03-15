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

#include <iim42652.h>
#include <zephyr/device.h>
#include <zephyr/display/cfb.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sensor_module, LOG_LEVEL_DBG);
static const struct device* const iim42652_dev = DEVICE_DT_GET(DT_NODELABEL(iim42652));
// display driver
static const struct device* display_dev = DEVICE_DT_GET(DT_NODELABEL(ssd1306));
// Display parameters
#define MAX_FONTS 42
#define SELECTED_FONT_INDEX 0
static uint16_t rows = 0;
static uint8_t ppt = 0;
static uint8_t font_width = 0;
static uint8_t font_height = 0;

#define IIM42652_TIMING_STARTUP 400  // 400ms
int main(void) {
#if Z_DEVICE_DT_FLAGS(DT_NODELABEL(iim42652)) & DEVICE_FLAG_INIT_DEFERRED
    LOG_DBG("Deferred init enabled, sleeping for %d ms", IIM42652_TIMING_STARTUP);
    k_sleep(K_MSEC(IIM42652_TIMING_STARTUP));
    device_init(iim42652_dev);
#endif
    if (!device_is_ready(iim42652_dev)) {
        LOG_ERR("Device %s is not ready\n", iim42652_dev->name);
        return -ENODEV;
    }
    LOG_DBG("IIM42652 Device %s is ready\n", iim42652_dev->name);
    // Starting display
    if (!device_is_ready(display_dev)) {
        LOG_ERR("Display device not ready\n");
        return -ENODEV;
    }

    if (display_set_pixel_format(display_dev, PIXEL_FORMAT_MONO01) != 0) {
        LOG_ERR("Failed to set required pixel format");
        return -ENOTSUP;
    }

    if (cfb_framebuffer_init(display_dev)) {
        LOG_ERR("Framebuffer init failed\n");
        return -EIO;
    }

    cfb_framebuffer_clear(display_dev, true);

    display_blanking_off(display_dev);

    rows = cfb_get_display_parameter(display_dev, CFB_DISPLAY_ROWS);
    ppt = cfb_get_display_parameter(display_dev, CFB_DISPLAY_PPT);

    for (int idx = 0; idx < MAX_FONTS; idx++) {
        if (cfb_get_font_size(display_dev, idx, &font_width, &font_height)) {
            break;  // end of font list, so exit loop.
        }

        LOG_DBG("index[%d] font width %d, font height %d", idx, font_width, font_height);
    }

    cfb_framebuffer_set_font(display_dev, SELECTED_FONT_INDEX);

    cfb_framebuffer_invert(display_dev);  // Optional: Invert the display (bright text on dark background)

    while (1) {
        if (NULL != iim42652_dev) {
            sensor_sample_fetch(iim42652_dev);
            struct sensor_value acc[3];
            sensor_channel_get(iim42652_dev, SENSOR_CHAN_ACCEL_XYZ, acc);
            double ax = acc[0].val1 + acc[0].val2 / 1000000.0;
            double ay = acc[1].val1 + acc[1].val2 / 1000000.0;
            double az = acc[2].val1 + acc[2].val2 / 1000000.0;
            LOG_DBG("Accel X: %.3f g, Y: %.3f g, Z: %.3f g", ax, ay, az);
            char buffer[64];
            snprintf(buffer, sizeof(buffer), "Accel X: %.3f g, Y: %.3f g, Z: %.3f g", ax, ay, az);

            cfb_print(display_dev, buffer, 0, 0);  // Print at
            cfb_framebuffer_finalize(display_dev);
        } else {
            LOG_ERR("IIM42652 device is NULL!\n");
        }
        k_sleep(K_MSEC(500));
    }
}
