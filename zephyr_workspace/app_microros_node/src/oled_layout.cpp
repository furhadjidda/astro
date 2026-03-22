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

void OLEDLayout::display_mtk3333_time(const char* time_str) { print_at_position(time_str, MTK_TIME_X, MTK_TIME_Y); }

void OLEDLayout::display_mtk3333_satellites(int count) {
    char buffer[32] = {0};
    snprintf(buffer, sizeof(buffer), "[%d]", count);
    print_at_position(buffer, MTK_SAT_X, MTK_SAT_Y);
}

void OLEDLayout::display_mtk3333_location(double latitude, double longitude, double altitude) {
    char buffer[128] = {0};
    snprintf(buffer, sizeof(buffer), "MTK3333: Lat=%.6f Lon=%.6f Alt=%.2f", latitude, longitude, altitude);
    // This may be logged or displayed depending on requirements
    // For now, just storing the format for reference
}

void OLEDLayout::display_ublox_time(const char* time_str) { print_at_position(time_str, UBLOX_TIME_X, UBLOX_TIME_Y); }

void OLEDLayout::display_ublox_satellites(int count) {
    char buffer[32] = {0};
    snprintf(buffer, sizeof(buffer), "[%d]", count);
    print_at_position(buffer, UBLOX_SAT_X, UBLOX_SAT_Y);
}

void OLEDLayout::display_ublox_location(double latitude, double longitude, double altitude) {
    char buffer[128] = {0};
    snprintf(buffer, sizeof(buffer), "Ublox: Lat=%.6f Lon=%.6f Alt=%.2f", latitude, longitude, altitude);
    // This may be logged or displayed depending on requirements
    // For now, just storing the format for reference
}

void OLEDLayout::display_imu_orientation(double x, double y, double z) {
    char buffer[64] = {0};

    snprintf(buffer, sizeof(buffer), "X=%.3f", x);
    print_at_position(buffer, IMU_LABEL_X, IMU_X_Y);

    snprintf(buffer, sizeof(buffer), "Y=%.3f", y);
    print_at_position(buffer, IMU_LABEL_X, IMU_Y_Y);

    snprintf(buffer, sizeof(buffer), "Z=%.3f", z);
    print_at_position(buffer, IMU_LABEL_X, IMU_Z_Y);
}

void OLEDLayout::display_temperature_pressure(int temp_val1, int temp_val2, int press_val1, int press_val2) {
    char buffer[64] = {0};

    snprintf(buffer, sizeof(buffer), "Temp&Pres");
    print_at_position(buffer, TEMP_PRESS_LABEL_X, TEMP_PRESS_LABEL_Y);

    snprintf(buffer, sizeof(buffer), "%d.%02dC", temp_val1, temp_val2 >= 0 ? temp_val2 / 10000 : -temp_val2 / 10000);
    print_at_position(buffer, TEMP_VAL_X, TEMP_VAL_Y);

    snprintf(buffer, sizeof(buffer), "%d.%02dkPa", press_val1,
             press_val2 >= 0 ? press_val2 / 10000 : -press_val2 / 10000);
    print_at_position(buffer, PRESS_VAL_X, PRESS_VAL_Y);
}

void OLEDLayout::display_status_message(const char* message) { print_at_position(message, 0, 0); }

void OLEDLayout::draw_default_grid() {
    if (oled_wrapper != nullptr) {
        oled_wrapper->draw_horizontal_line(15, 0, 128);
        oled_wrapper->draw_vertical_line(64, 0, 64);
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
