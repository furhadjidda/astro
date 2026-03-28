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

constexpr uint8_t SATELLITE_ICON_14X10[] = {
    0b00001111, 0b00000000, 0b11011111, 0b11101100, 0b11011111, 0b11101100, 0b11111111,
    0b11111100, 0b11011111, 0b11101100, 0b11001111, 0b11001100, 0b00000111, 0b10000000,
    0b00001111, 0b11000000, 0b00011100, 0b11100000, 0b00110000, 0b00110000,
};

constexpr uint8_t IMU_AXIS_ICON_16X16[] = {
    0x00, 0x40, 0x00, 0xE0, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x46,
    0x00, 0xFF, 0x00, 0xC6, 0x01, 0x00, 0x02, 0x00, 0x04, 0x00, 0x28, 0x00, 0x38, 0x00, 0x10, 0x00,
};

constexpr uint8_t TEMP_PRESS_ICON_16X16[] = {
    0b00000110, 0b00000000, 0b00001111, 0b00011000, 0b00001111, 0b00111100, 0b00001111, 0b00011000,
    0b00001111, 0b00000000, 0b00001111, 0b00011000, 0b00001111, 0b00111100, 0b00001111, 0b01111110,
    0b00001111, 0b00111100, 0b00011111, 0b00011000, 0b00011111, 0b00000000, 0b00111111, 0b10000000,
    0b00111111, 0b10000000, 0b00111111, 0b10000000, 0b00011111, 0b00000000, 0b00001111, 0b00000000,
};

constexpr uint8_t WIFI_ICON_24X16[] = {
    0b00000000, 0b01111110, 0b00000000, 0b00000001, 0b11111111, 0b00000000, 0b00000011, 0b10000001,
    0b10000000, 0b00000110, 0b00000000, 0b11000000, 0b00001100, 0b01111110, 0b01100000, 0b00011000,
    0b11111111, 0b00110000, 0b00110001, 0b10000001, 0b10011000, 0b01100011, 0b00000000, 0b11001100,
    0b00000110, 0b00000000, 0b01100000, 0b00001100, 0b00111100, 0b00110000, 0b00000000, 0b01111110,
    0b00000000, 0b00000000, 0b00011000, 0b00000000, 0b00000000, 0b00111100, 0b00000000, 0b00000000,
    0b00111100, 0b00000000, 0b00000000, 0b00011000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
};

constexpr uint8_t WAITING_ICON_16X16[] = {
    0b00000111, 0b11100000, 0b00011000, 0b00011000, 0b00100000, 0b00000100, 0b01000011, 0b11000010,
    0b01000100, 0b00100010, 0b10001000, 0b00010001, 0b10001000, 0b00010001, 0b10001000, 0b00010001,
    0b10001000, 0b11010001, 0b10001000, 0b01010001, 0b10001000, 0b00110001, 0b01000100, 0b00000010,
    0b01000011, 0b00000100, 0b00100000, 0b00001000, 0b00011000, 0b00110000, 0b00000111, 0b11000000,
};
}  // namespace

OLEDLayout::OLEDLayout(OLEDWrapper* oled_wrapper) : oled_wrapper(oled_wrapper) { k_mutex_init(&display_mutex); }

int OLEDLayout::init_screen() {
    if (oled_wrapper == nullptr) {
        return -ENODEV;
    }
    return oled_wrapper->init();
}

void OLEDLayout::set_display_updates_enabled(bool enabled) {
    k_mutex_lock(&display_mutex, K_FOREVER);
    display_updates_enabled = enabled;
    k_mutex_unlock(&display_mutex);
}

void OLEDLayout::print_at_position(const char* text, int x, int y) {
    if (oled_wrapper != nullptr && text != nullptr) {
        oled_wrapper->print(text, x, y);
    }
}

bool OLEDLayout::is_view_active(View view) const { return active_view == view; }

void OLEDLayout::set_view(View view) {
    k_mutex_lock(&display_mutex, K_FOREVER);
    active_view = view;
    if (!display_updates_enabled) {
        k_mutex_unlock(&display_mutex);
        return;
    }
    show_active_view_locked();
    k_mutex_unlock(&display_mutex);
}

void OLEDLayout::next_view() {
    k_mutex_lock(&display_mutex, K_FOREVER);
    const uint8_t current = static_cast<uint8_t>(active_view);
    const uint8_t next = static_cast<uint8_t>((current + 1U) % 3U);
    active_view = static_cast<View>(next);
    if (!display_updates_enabled) {
        k_mutex_unlock(&display_mutex);
        return;
    }
    show_active_view_locked();
    k_mutex_unlock(&display_mutex);
}

OLEDLayout::View OLEDLayout::get_view() const { return active_view; }

void OLEDLayout::show_active_view() {
    k_mutex_lock(&display_mutex, K_FOREVER);
    show_active_view_locked();
    k_mutex_unlock(&display_mutex);
}

void OLEDLayout::show_active_view_locked() {
    if (oled_wrapper == nullptr || !display_updates_enabled) {
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
    draw_satellite_icon(27, SAT_HEADER_Y + 1);

    print_at_position("UBX", SAT_SPLIT_X + 2, SAT_HEADER_Y);
    draw_satellite_icon(SAT_SPLIT_X + 33, SAT_HEADER_Y + 1);

    draw_clock_icon(3, SAT_TIME_Y + 1);
    draw_clock_icon(SAT_SPLIT_X + 9, SAT_TIME_Y + 1);

    print_at_position("Lt:", 2, SAT_LAT_Y);
    print_at_position("Ln:", 2, SAT_LON_Y);
    print_at_position("Al:", 2, SAT_ALT_Y);

    print_at_position("Lt:", SAT_SPLIT_X + 8, SAT_LAT_Y);
    print_at_position("Ln:", SAT_SPLIT_X + 8, SAT_LON_Y);
    print_at_position("Al:", SAT_SPLIT_X + 8, SAT_ALT_Y);
}

void OLEDLayout::draw_temp_pressure_view_layout() {
    print_at_position("TEMP & PRESS", TP_TITLE_X, TP_TITLE_Y);
    draw_temp_pressure_icon(2, TP_TEMP_Y - 4);
}

void OLEDLayout::draw_imu_view_layout() {
    print_at_position("IMU", IMU_TITLE_X, IMU_TITLE_Y);
    draw_imu_axis_icon(IMU_AXIS_X, IMU_AXIS_Y);
}

void OLEDLayout::render_satellite_values() {
    char buffer[16] = {0};

    print_at_position("     ", 14, SAT_TIME_Y);
    print_at_position(mtk_time_cache, 14, SAT_TIME_Y);
    print_at_position("     ", SAT_SPLIT_X + 18, SAT_TIME_Y);
    print_at_position(ublox_time_cache, SAT_SPLIT_X + 18, SAT_TIME_Y);

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
    print_at_position("     ", SAT_SPLIT_X + 18, SAT_COUNT_Y);
    print_at_position(buffer, SAT_SPLIT_X + 18, SAT_COUNT_Y);

    if (has_mtk_location_cache) {
        snprintf(buffer, sizeof(buffer), "%+0.1f", mtk_lat_cache);
        print_at_position("      ", 20, SAT_LAT_Y);
        print_at_position(buffer, 20, SAT_LAT_Y);

        snprintf(buffer, sizeof(buffer), "%+0.1f", mtk_lon_cache);
        print_at_position("      ", 20, SAT_LON_Y);
        print_at_position(buffer, 20, SAT_LON_Y);

        snprintf(buffer, sizeof(buffer), "%0.0fm", mtk_alt_cache);
        print_at_position("      ", 20, SAT_ALT_Y);
        print_at_position(buffer, 20, SAT_ALT_Y);
    } else {
        print_at_position("--.-  ", 20, SAT_LAT_Y);
        print_at_position("--.-  ", 20, SAT_LON_Y);
        print_at_position("---m  ", 20, SAT_ALT_Y);
    }

    if (has_ublox_location_cache) {
        snprintf(buffer, sizeof(buffer), "%+0.1f", ublox_lat_cache);
        print_at_position("      ", SAT_SPLIT_X + 24, SAT_LAT_Y);
        print_at_position(buffer, SAT_SPLIT_X + 24, SAT_LAT_Y);

        snprintf(buffer, sizeof(buffer), "%+0.1f", ublox_lon_cache);
        print_at_position("      ", SAT_SPLIT_X + 24, SAT_LON_Y);
        print_at_position(buffer, SAT_SPLIT_X + 24, SAT_LON_Y);

        snprintf(buffer, sizeof(buffer), "%0.0fm", ublox_alt_cache);
        print_at_position("      ", SAT_SPLIT_X + 24, SAT_ALT_Y);
        print_at_position(buffer, SAT_SPLIT_X + 24, SAT_ALT_Y);
    } else {
        print_at_position("--.-  ", SAT_SPLIT_X + 24, SAT_LAT_Y);
        print_at_position("--.-  ", SAT_SPLIT_X + 24, SAT_LON_Y);
        print_at_position("---m  ", SAT_SPLIT_X + 24, SAT_ALT_Y);
    }
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
    k_mutex_lock(&display_mutex, K_FOREVER);
    if (time_str != nullptr) {
        snprintf(mtk_time_cache, sizeof(mtk_time_cache), "%s", time_str);
    }

    if (!display_updates_enabled || !is_view_active(View::SATELLITE)) {
        k_mutex_unlock(&display_mutex);
        return;
    }

    render_satellite_values();
    oled_wrapper->finalize();
    k_mutex_unlock(&display_mutex);
}

void OLEDLayout::display_mtk3333_satellites(int count) {
    k_mutex_lock(&display_mutex, K_FOREVER);
    mtk_sat_cache = count;

    if (!display_updates_enabled || !is_view_active(View::SATELLITE)) {
        k_mutex_unlock(&display_mutex);
        return;
    }

    render_satellite_values();
    oled_wrapper->finalize();
    k_mutex_unlock(&display_mutex);
}

void OLEDLayout::display_mtk3333_location(double latitude, double longitude, double altitude) {
    k_mutex_lock(&display_mutex, K_FOREVER);
    mtk_lat_cache = latitude;
    mtk_lon_cache = longitude;
    mtk_alt_cache = altitude;
    has_mtk_location_cache = true;

    if (!display_updates_enabled || !is_view_active(View::SATELLITE)) {
        k_mutex_unlock(&display_mutex);
        return;
    }

    render_satellite_values();
    oled_wrapper->finalize();
    k_mutex_unlock(&display_mutex);
}

void OLEDLayout::display_ublox_time(const char* time_str) {
    k_mutex_lock(&display_mutex, K_FOREVER);
    if (time_str != nullptr) {
        snprintf(ublox_time_cache, sizeof(ublox_time_cache), "%s", time_str);
    }

    if (!display_updates_enabled || !is_view_active(View::SATELLITE)) {
        k_mutex_unlock(&display_mutex);
        return;
    }

    render_satellite_values();
    oled_wrapper->finalize();
    k_mutex_unlock(&display_mutex);
}

void OLEDLayout::display_ublox_satellites(int count) {
    k_mutex_lock(&display_mutex, K_FOREVER);
    ublox_sat_cache = count;

    if (!display_updates_enabled || !is_view_active(View::SATELLITE)) {
        k_mutex_unlock(&display_mutex);
        return;
    }

    render_satellite_values();
    oled_wrapper->finalize();
    k_mutex_unlock(&display_mutex);
}

void OLEDLayout::display_ublox_location(double latitude, double longitude, double altitude) {
    k_mutex_lock(&display_mutex, K_FOREVER);
    ublox_lat_cache = latitude;
    ublox_lon_cache = longitude;
    ublox_alt_cache = altitude;
    has_ublox_location_cache = true;

    if (!display_updates_enabled || !is_view_active(View::SATELLITE)) {
        k_mutex_unlock(&display_mutex);
        return;
    }

    render_satellite_values();
    oled_wrapper->finalize();
    k_mutex_unlock(&display_mutex);
}

void OLEDLayout::display_imu_orientation(double x, double y, double z) {
    k_mutex_lock(&display_mutex, K_FOREVER);
    imu_x_cache = x;
    imu_y_cache = y;
    imu_z_cache = z;
    has_imu_cache = true;

    if (!display_updates_enabled || !is_view_active(View::IMU)) {
        k_mutex_unlock(&display_mutex);
        return;
    }

    render_imu_values();
    oled_wrapper->finalize();
    k_mutex_unlock(&display_mutex);
}

void OLEDLayout::display_temperature_pressure(int temp_val1, int temp_val2, int press_val1, int press_val2) {
    k_mutex_lock(&display_mutex, K_FOREVER);
    temp_val1_cache = temp_val1;
    temp_val2_cache = temp_val2;
    press_val1_cache = press_val1;
    press_val2_cache = press_val2;
    has_temp_press_cache = true;

    if (!display_updates_enabled || !is_view_active(View::TEMP_PRESSURE)) {
        k_mutex_unlock(&display_mutex);
        return;
    }

    render_temp_pressure_values();
    oled_wrapper->finalize();
    k_mutex_unlock(&display_mutex);
}

void OLEDLayout::display_status_message(const char* message) {
    k_mutex_lock(&display_mutex, K_FOREVER);
    if (oled_wrapper == nullptr) {
        k_mutex_unlock(&display_mutex);
        return;
    }
    oled_wrapper->clear();
    print_at_position(message, 0, 0);
    oled_wrapper->finalize();
    k_mutex_unlock(&display_mutex);
}

void OLEDLayout::display_wifi_waiting_message(const char* ssid) {
    k_mutex_lock(&display_mutex, K_FOREVER);
    if (oled_wrapper == nullptr) {
        k_mutex_unlock(&display_mutex);
        return;
    }

    char line1[32] = {0};
    char line2[32] = {0};

    if (ssid != nullptr && ssid[0] != '\0') {
        snprintf(line1, sizeof(line1), "Waiting for Wifi");
        snprintf(line2, sizeof(line2), "'%s'...", ssid);
    } else {
        snprintf(line1, sizeof(line1), "Waiting for Wifi...");
    }

    oled_wrapper->clear();
    draw_wifi_icon(52, 6);
    print_at_position(line1, 12, 40);
    if (line2[0] != '\0') {
        print_at_position(line2, 26, 52);
    }
    oled_wrapper->finalize();
    k_mutex_unlock(&display_mutex);
}

void OLEDLayout::display_agent_waiting_message() {
    k_mutex_lock(&display_mutex, K_FOREVER);
    if (oled_wrapper == nullptr) {
        k_mutex_unlock(&display_mutex);
        return;
    }

    oled_wrapper->clear();
    draw_waiting_icon(56, 8);
    print_at_position("Waiting for", 28, 42);
    print_at_position("ROS Agent", 34, 54);
    oled_wrapper->finalize();
    k_mutex_unlock(&display_mutex);
}

void OLEDLayout::show_startup_splash() {
    k_mutex_lock(&display_mutex, K_FOREVER);
    if (oled_wrapper == nullptr) {
        k_mutex_unlock(&display_mutex);
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
    k_mutex_unlock(&display_mutex);
}

void OLEDLayout::draw_default_grid() {
    k_mutex_lock(&display_mutex, K_FOREVER);
    if (oled_wrapper != nullptr && display_updates_enabled) {
        show_active_view_locked();
    }
    k_mutex_unlock(&display_mutex);
}

void OLEDLayout::clear_screen() {
    k_mutex_lock(&display_mutex, K_FOREVER);
    if (oled_wrapper != nullptr) {
        oled_wrapper->clear();
    }
    k_mutex_unlock(&display_mutex);
}

void OLEDLayout::finalize_screen() {
    k_mutex_lock(&display_mutex, K_FOREVER);
    if (oled_wrapper != nullptr) {
        oled_wrapper->finalize();
    }
    k_mutex_unlock(&display_mutex);
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

void OLEDLayout::draw_satellite_icon(int x, int y) { draw_bitmap_1bpp(SATELLITE_ICON_14X10, 14, 10, x, y); }

void OLEDLayout::draw_clock_icon(int x, int y) { draw_bitmap_1bpp(CLOCK_ICON_8X8, 8, 8, x, y); }

void OLEDLayout::draw_imu_axis_icon(int x, int y) { draw_bitmap_1bpp(IMU_AXIS_ICON_16X16, 16, 16, x, y); }

void OLEDLayout::draw_temp_pressure_icon(int x, int y) { draw_bitmap_1bpp(TEMP_PRESS_ICON_16X16, 16, 16, x, y); }

void OLEDLayout::draw_wifi_icon(int x, int y) { draw_bitmap_1bpp(WIFI_ICON_24X16, 24, 16, x, y); }

void OLEDLayout::draw_waiting_icon(int x, int y) { draw_bitmap_1bpp(WAITING_ICON_16X16, 16, 16, x, y); }
