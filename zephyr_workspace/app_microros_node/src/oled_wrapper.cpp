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

#include "oled_wrapper.hpp"

// Display parameters
#define MAX_FONTS 42

LOG_MODULE_REGISTER(oled_wrapper, LOG_LEVEL_DBG);

OLEDWrapper::OLEDWrapper(const struct device* display_dev) : display_dev(display_dev) {}

int OLEDWrapper::init() {
    if (!device_is_ready(display_dev)) {
        LOG_ERR("Display device not ready\n");
        return -ENODEV;
    }

    if (display_set_pixel_format(display_dev, PIXEL_FORMAT_MONO01) != 0) {
        LOG_ERR("Failed to set required pixel format");
        return -EIO;
    }

    if (cfb_framebuffer_init(display_dev)) {
        LOG_ERR("Framebuffer init failed\n");
        return -EIO;
    }
    _rows = cfb_get_display_parameter(display_dev, CFB_DISPLAY_ROWS);
    _ppt = cfb_get_display_parameter(display_dev, CFB_DISPLAY_PPT);

    int best_font_idx = 0;
    uint32_t best_area = UINT32_MAX;

    for (int idx = 0; idx < MAX_FONTS; idx++) {
        uint8_t w = 0, h = 0;
        if (cfb_get_font_size(display_dev, idx, &w, &h)) {
            break;  // end of font list, so exit loop.
        }

        LOG_DBG("index[%d] font width %d, font height %d", idx, w, h);

        uint32_t area = (uint32_t)w * h;
        if (area < best_area) {
            best_area = area;
            best_font_idx = idx;
            _font_width = w;
            _font_height = h;
        }
    }

    LOG_DBG("Selected smallest font index[%d] width %d, height %d", best_font_idx, _font_width, _font_height);
    cfb_framebuffer_set_font(display_dev, best_font_idx);

    cfb_framebuffer_invert(display_dev);  // Optional: Invert the display (bright text on dark background)

    return 0;
}

void OLEDWrapper::print(const char* message, int x, int y) { cfb_print(display_dev, message, x, y); }

void OLEDWrapper::clear() { cfb_framebuffer_clear(display_dev, true); }

void OLEDWrapper::finalize() { cfb_framebuffer_finalize(display_dev); }

void OLEDWrapper::draw_vertical_line(int x, int y_start, int y_end) {
    struct cfb_position start = {static_cast<uint16_t>(x), static_cast<uint16_t>(y_start)};
    struct cfb_position end = {static_cast<uint16_t>(x), static_cast<uint16_t>(y_end)};
    cfb_draw_line(display_dev, &start, &end);
}

void OLEDWrapper::draw_horizontal_line(int y, int x_start, int x_end) {
    struct cfb_position start = {static_cast<uint16_t>(x_start), static_cast<uint16_t>(y)};
    struct cfb_position end = {static_cast<uint16_t>(x_end), static_cast<uint16_t>(y)};
    cfb_draw_line(display_dev, &start, &end);
}