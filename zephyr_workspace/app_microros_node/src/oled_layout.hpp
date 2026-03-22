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

#ifndef OLED_LAYOUT_HPP
#define OLED_LAYOUT_HPP

#include "oled_wrapper.hpp"

/**
 * OLEDLayout manages the OLED screen layout and positioning.
 * It provides high-level semantic methods for displaying different types of data.
 * All position calculations are handled internally, so callers only need to specify what to display.
 *
 * Screen Layout:
 * +--------+--------+
 * | MTK333 | uBlox  |  <- Time display
 * | [n]    | [m]    |  <- Satellite counts
 * +--------+--------+
 * | IMU    | Temp   |
 * | X:val  | & Pres |
 * | Y:val  | T:val  |
 * | Z:val  | P:val  |
 * +--------+--------+
 */
class OLEDLayout {
   public:
    OLEDLayout(OLEDWrapper* oled_wrapper);

    int init_screen();

    // Layout regions
    void display_mtk3333_time(const char* time_str);
    void display_mtk3333_satellites(int count);
    void display_mtk3333_location(double latitude, double longitude, double altitude);

    void display_ublox_time(const char* time_str);
    void display_ublox_satellites(int count);
    void display_ublox_location(double latitude, double longitude, double altitude);

    void display_imu_orientation(double x, double y, double z);

    void display_temperature_pressure(int temp_val1, int temp_val2, int press_val1, int press_val2);

    void display_status_message(const char* message);
    void draw_default_grid();

    void clear_screen();
    void finalize_screen();

   private:
    OLEDWrapper* oled_wrapper;

    // Screen layout constants (in pixels)
    static constexpr int SCREEN_WIDTH = 128;
    static constexpr int SCREEN_HEIGHT = 64;
    static constexpr int LEFT_COLUMN_WIDTH = 64;
    static constexpr int RIGHT_COLUMN_WIDTH = 64;

    // Left column (MTK3333) regions
    static constexpr int MTK_TIME_X = 68;
    static constexpr int MTK_TIME_Y = 0;
    static constexpr int MTK_SAT_X = 110;
    static constexpr int MTK_SAT_Y = 2;

    // Right column (uBlox) regions
    static constexpr int UBLOX_TIME_X = 0;
    static constexpr int UBLOX_TIME_Y = 0;
    static constexpr int UBLOX_SAT_X = 35;
    static constexpr int UBLOX_SAT_Y = 2;

    // IMU region (left side)
    static constexpr int IMU_LABEL_X = 0;
    static constexpr int IMU_LABEL_Y = 20;
    static constexpr int IMU_X_Y = 20;
    static constexpr int IMU_X_VAL_Y = 20;
    static constexpr int IMU_Y_Y = 30;
    static constexpr int IMU_Y_VAL_Y = 30;
    static constexpr int IMU_Z_Y = 40;
    static constexpr int IMU_Z_VAL_Y = 40;

    // Temperature & Pressure region (right side)
    static constexpr int TEMP_PRESS_LABEL_X = 70;
    static constexpr int TEMP_PRESS_LABEL_Y = 20;
    static constexpr int TEMP_VAL_X = 70;
    static constexpr int TEMP_VAL_Y = 35;
    static constexpr int PRESS_VAL_X = 70;
    static constexpr int PRESS_VAL_Y = 45;

    // Helper to print formatted text at a position
    void print_at_position(const char* text, int x, int y);
};

#endif  // OLED_LAYOUT_HPP
