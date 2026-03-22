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

#include "oled_layout.hpp"

#include <errno.h>

#include <cstdio>
#include <cstring>

namespace {
constexpr uint8_t CLOCK_ICON_8X8[] = {
    0b00111100, 0b01000010, 0b10011001, 0b10100001, 0b10111101, 0b10000001, 0b01000010, 0b00111100,
};

constexpr uint8_t SATELLITE_ICON_10X8[] = {
    0b00110110, 0b00000000, 0b00011100, 0b00000000, 0b11111111, 0b11000000, 0b00111111, 0b00000000,
    0b11111111, 0b11000000, 0b00011100, 0b00000000, 0b00110110, 0b00000000, 0b00000000, 0b00000000,
};

constexpr uint8_t IMU_AXIS_ICON_12X12[] = {
    0x04, 0x00, 0x04, 0x00, 0x04, 0x00, 0x04, 0x00, 0x04, 0xC0, 0xFF, 0xE0,
    0x04, 0xC0, 0x04, 0x00, 0x04, 0x00, 0x04, 0x00, 0x04, 0x00, 0x00, 0x00,
};

constexpr uint8_t TEMP_PRESS_ICON_10X12[] = {
    0b00110000, 0b00000000, 0b00110000, 0b00000000, 0b00110000, 0b00000000, 0b00110011, 0b00000000,
    0b00110011, 0b00000000, 0b00111111, 0b00000000, 0b00111111, 0b00000000, 0b00111111, 0b00000000,
    0b00001111, 0b00000000, 0b00000110, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
};
}  // namespace

OLEDLayout::OLEDLayout(OLEDWrapper* oled_wrapper) : oled_wrapper(oled_wrapper) {}

int OLEDLayout::init_screen() {
    if (oled_wrapper == nullptr) {
        return -ENODEV;
    }
    return oled_wrapper->init();
}

void OLEDLayout::print_at_position(const char* text, int x, int y) {
    if (oled_wrapper != nullptr && text != nullptr) {
        oled_wrapper->print(text, x, y);
    }
}

bool OLEDLayout::is_view_active(View view) const { return active_view == view; }

void OLEDLayout::set_view(View view) {
    active_view = view;
    show_active_view();
}

void OLEDLayout::next_view() {
    const uint8_t current = static_cast<uint8_t>(active_view);
    const uint8_t next = static_cast<uint8_t>((current + 1U) % 3U);
    active_view = static_cast<View>(next);
    show_active_view();
}

OLEDLayout::View OLEDLayout::get_view() const { return active_view; }

void OLEDLayout::show_active_view() {
    if (oled_wrapper == nullptr) {
        return;
    }

    oled_wrapper->clear();

    if (active_view == View::SATELLITE) {
        draw_satellite_view_layout();
        render_satellite_values();
    } else if (active_view == View::TEMP_PRESSURE) {
        draw_temp_pressure_view_layout();
        render_temp_pressure_values();
    } else {
        draw_imu_view_layout();
        render_imu_values();
    }

    oled_wrapper->finalize();
}

void OLEDLayout::draw_satellite_view_layout() {
    // Vertical partition: left MTK3333, right uBlox.
    oled_wrapper->draw_vertical_line(SAT_SPLIT_X, 0, SCREEN_HEIGHT - 1);

    print_at_position("MTK", 3, SAT_HEADER_Y);
    draw_satellite_icon(31, SAT_HEADER_Y + 2);

    print_at_position("UBX", SAT_SPLIT_X + 3, SAT_HEADER_Y);
    draw_satellite_icon(SAT_SPLIT_X + 31, SAT_HEADER_Y + 2);

    draw_clock_icon(3, SAT_TIME_Y + 1);
    draw_clock_icon(SAT_SPLIT_X + 3, SAT_TIME_Y + 1);
}

void OLEDLayout::draw_temp_pressure_view_layout() {
    print_at_position("TEMP & PRESS", TP_TITLE_X, TP_TITLE_Y);
    draw_temp_pressure_icon(2, TP_TEMP_Y - 2);
}

void OLEDLayout::draw_imu_view_layout() {
    print_at_position("IMU", IMU_TITLE_X, IMU_TITLE_Y);
    draw_imu_axis_icon(IMU_AXIS_X, IMU_AXIS_Y);
}

void OLEDLayout::render_satellite_values() {
    char buffer[16] = {0};

    print_at_position("     ", 14, SAT_TIME_Y);
    print_at_position(mtk_time_cache, 14, SAT_TIME_Y);
    print_at_position("     ", SAT_SPLIT_X + 14, SAT_TIME_Y);
    print_at_position(ublox_time_cache, SAT_SPLIT_X + 14, SAT_TIME_Y);

    if (mtk_sat_cache >= 0) {
        snprintf(buffer, sizeof(buffer), "[%d]", mtk_sat_cache);
    } else {
        snprintf(buffer, sizeof(buffer), "[--]");
    }
    print_at_position("     ", 14, SAT_COUNT_Y);
    print_at_position(buffer, 14, SAT_COUNT_Y);

    if (ublox_sat_cache >= 0) {
        snprintf(buffer, sizeof(buffer), "[%d]", ublox_sat_cache);
    } else {
        snprintf(buffer, sizeof(buffer), "[--]");
    }
    print_at_position("     ", SAT_SPLIT_X + 14, SAT_COUNT_Y);
    print_at_position(buffer, SAT_SPLIT_X + 14, SAT_COUNT_Y);
}

void OLEDLayout::render_temp_pressure_values() {
    char buffer[32] = {0};

    if (!has_temp_press_cache) {
        print_at_position("T: --.-- C", TP_TEMP_X, TP_TEMP_Y);
        print_at_position("P: ----.--k", TP_PRESS_X, TP_PRESS_Y);
        return;
    }

    snprintf(buffer, sizeof(buffer), "T:%d.%02d C", temp_val1_cache,
             temp_val2_cache >= 0 ? temp_val2_cache / 10000 : -temp_val2_cache / 10000);
    print_at_position("           ", TP_TEMP_X, TP_TEMP_Y);
    print_at_position(buffer, TP_TEMP_X, TP_TEMP_Y);

    snprintf(buffer, sizeof(buffer), "P:%d.%02dk", press_val1_cache,
             press_val2_cache >= 0 ? press_val2_cache / 10000 : -press_val2_cache / 10000);
    print_at_position("           ", TP_PRESS_X, TP_PRESS_Y);
    print_at_position(buffer, TP_PRESS_X, TP_PRESS_Y);
}

void OLEDLayout::render_imu_values() {
    char buffer[32] = {0};

    if (!has_imu_cache) {
        print_at_position("X=+0.000", IMU_ONLY_X, IMU_ONLY_X_Y);
        print_at_position("Y=+0.000", IMU_ONLY_X, IMU_ONLY_Y_Y);
        print_at_position("Z=+0.000", IMU_ONLY_X, IMU_ONLY_Z_Y);
        return;
    }

    snprintf(buffer, sizeof(buffer), "X=%+0.3f", imu_x_cache);
    print_at_position("        ", IMU_ONLY_X, IMU_ONLY_X_Y);
    print_at_position(buffer, IMU_ONLY_X, IMU_ONLY_X_Y);

    snprintf(buffer, sizeof(buffer), "Y=%+0.3f", imu_y_cache);
    print_at_position("        ", IMU_ONLY_X, IMU_ONLY_Y_Y);
    print_at_position(buffer, IMU_ONLY_X, IMU_ONLY_Y_Y);

    snprintf(buffer, sizeof(buffer), "Z=%+0.3f", imu_z_cache);
    print_at_position("        ", IMU_ONLY_X, IMU_ONLY_Z_Y);
    print_at_position(buffer, IMU_ONLY_X, IMU_ONLY_Z_Y);
}

void OLEDLayout::display_mtk3333_time(const char* time_str) {
    if (time_str != nullptr) {
        snprintf(mtk_time_cache, sizeof(mtk_time_cache), "%s", time_str);
    }

    if (!is_view_active(View::SATELLITE)) {
        return;
    }

    render_satellite_values();
}

void OLEDLayout::display_mtk3333_satellites(int count) {
    mtk_sat_cache = count;

    if (!is_view_active(View::SATELLITE)) {
        return;
    }

    render_satellite_values();
}

void OLEDLayout::display_mtk3333_location(double latitude, double longitude, double altitude) {
    char buffer[128] = {0};
    snprintf(buffer, sizeof(buffer), "MTK3333: Lat=%.6f Lon=%.6f Alt=%.2f", latitude, longitude, altitude);
    // This may be logged or displayed depending on requirements
    // For now, just storing the format for reference
}

void OLEDLayout::display_ublox_time(const char* time_str) {
    if (time_str != nullptr) {
        snprintf(ublox_time_cache, sizeof(ublox_time_cache), "%s", time_str);
    }

    if (!is_view_active(View::SATELLITE)) {
        return;
    }

    render_satellite_values();
}

void OLEDLayout::display_ublox_satellites(int count) {
    ublox_sat_cache = count;

    if (!is_view_active(View::SATELLITE)) {
        return;
    }

    render_satellite_values();
}

void OLEDLayout::display_ublox_location(double latitude, double longitude, double altitude) {
    char buffer[128] = {0};
    snprintf(buffer, sizeof(buffer), "Ublox: Lat=%.6f Lon=%.6f Alt=%.2f", latitude, longitude, altitude);
    // This may be logged or displayed depending on requirements
    // For now, just storing the format for reference
}

void OLEDLayout::display_imu_orientation(double x, double y, double z) {
    imu_x_cache = x;
    imu_y_cache = y;
    imu_z_cache = z;
    has_imu_cache = true;

    if (!is_view_active(View::IMU)) {
        return;
    }

    render_imu_values();
}

void OLEDLayout::display_temperature_pressure(int temp_val1, int temp_val2, int press_val1, int press_val2) {
    temp_val1_cache = temp_val1;
    temp_val2_cache = temp_val2;
    press_val1_cache = press_val1;
    press_val2_cache = press_val2;
    has_temp_press_cache = true;

    if (!is_view_active(View::TEMP_PRESSURE)) {
        return;
    }

    render_temp_pressure_values();
}

void OLEDLayout::display_status_message(const char* message) { print_at_position(message, 0, 0); }

void OLEDLayout::show_startup_splash() {
    if (oled_wrapper == nullptr) {
        return;
    }

    // Monochrome splash inspired by astro.jpeg (rover body, wheels, and logo mark).
    oled_wrapper->clear();

    // Rover body
    oled_wrapper->draw_rectangle(18, 9, 110, 37);
    oled_wrapper->draw_horizontal_line(20, 24, 103);

    // Wheels
    oled_wrapper->draw_circle(29, 49, 10);
    oled_wrapper->draw_circle(56, 49, 10);
    oled_wrapper->draw_circle(82, 49, 10);
    oled_wrapper->draw_circle(108, 49, 10);
    oled_wrapper->draw_circle(29, 49, 4);
    oled_wrapper->draw_circle(56, 49, 4);
    oled_wrapper->draw_circle(82, 49, 4);
    oled_wrapper->draw_circle(108, 49, 4);

    // Axle and suspension bars
    oled_wrapper->draw_horizontal_line(43, 30, 107);
    oled_wrapper->draw_horizontal_line(45, 30, 107);

    // Antenna
    oled_wrapper->draw_vertical_line(100, 1, 9);
    oled_wrapper->draw_rectangle(98, 0, 102, 2);

    // Orbit mark on the body
    oled_wrapper->draw_circle(61, 20, 7);
    for (int i = 0; i < 13; ++i) {
        oled_wrapper->draw_point(55 + i, 26 - i / 2);
    }

    oled_wrapper->print("astro", 46, 28);
    oled_wrapper->finalize();
}

void OLEDLayout::draw_default_grid() {
    if (oled_wrapper != nullptr) {
        show_active_view();
    }
}

void OLEDLayout::clear_screen() {
    if (oled_wrapper != nullptr) {
        oled_wrapper->clear();
    }
}

void OLEDLayout::finalize_screen() {
    if (oled_wrapper != nullptr) {
        oled_wrapper->finalize();
    }
}

void OLEDLayout::draw_bitmap_1bpp(const uint8_t* bitmap, int width, int height, int x, int y) {
    if (oled_wrapper == nullptr || bitmap == nullptr) {
        return;
    }

    const int row_stride_bytes = (width + 7) / 8;
    for (int row = 0; row < height; ++row) {
        for (int col = 0; col < width; ++col) {
            const int byte_index = row * row_stride_bytes + (col / 8);
            const uint8_t bit_mask = static_cast<uint8_t>(0x80U >> (col % 8));
            if ((bitmap[byte_index] & bit_mask) != 0U) {
                oled_wrapper->draw_point(x + col, y + row);
            }
        }
    }
}

void OLEDLayout::draw_satellite_icon(int x, int y) { draw_bitmap_1bpp(SATELLITE_ICON_10X8, 10, 8, x, y); }

void OLEDLayout::draw_clock_icon(int x, int y) { draw_bitmap_1bpp(CLOCK_ICON_8X8, 8, 8, x, y); }

void OLEDLayout::draw_imu_axis_icon(int x, int y) { draw_bitmap_1bpp(IMU_AXIS_ICON_12X12, 12, 12, x, y); }

void OLEDLayout::draw_temp_pressure_icon(int x, int y) { draw_bitmap_1bpp(TEMP_PRESS_ICON_10X12, 10, 12, x, y); }
