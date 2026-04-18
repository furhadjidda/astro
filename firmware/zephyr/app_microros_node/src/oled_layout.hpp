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

#include <cstdint>

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
    enum class View : uint8_t {
        SATELLITE = 0,
        TEMP_PRESSURE = 1,
        IMU = 2,
    };

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
    void display_wifi_waiting_message(const char* ssid);
    void display_agent_waiting_message();
    void show_startup_splash();
    void set_display_updates_enabled(bool enabled);
    void set_view(View view);
    void next_view();
    View get_view() const;
    void show_active_view();
    void draw_default_grid();

    void clear_screen();
    void finalize_screen();

   private:
    OLEDWrapper* oled_wrapper;
    volatile View active_view = View::SATELLITE;
    bool display_updates_enabled = false;
    struct k_mutex display_mutex;

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
    static constexpr int TEMP_PRESS_ICON_X = 70;
    static constexpr int TEMP_PRESS_ICON_Y = 20;
    static constexpr int TEMP_VAL_X = 70;
    static constexpr int TEMP_VAL_Y = 35;
    static constexpr int PRESS_VAL_X = 70;
    static constexpr int PRESS_VAL_Y = 45;

    // Satellite-only view constants
    static constexpr int SAT_SPLIT_X = 57;
    static constexpr int SAT_HEADER_Y = 0;
    static constexpr int SAT_TIME_Y = 12;
    static constexpr int SAT_COUNT_Y = 22;
    static constexpr int SAT_LAT_Y = 32;
    static constexpr int SAT_LON_Y = 42;
    static constexpr int SAT_ALT_Y = 52;

    // Temperature/Pressure-only view constants
    static constexpr int TP_TITLE_X = 26;
    static constexpr int TP_TITLE_Y = 0;
    static constexpr int TP_TEMP_X = 24;
    static constexpr int TP_TEMP_Y = 24;
    static constexpr int TP_PRESS_X = 24;
    static constexpr int TP_PRESS_Y = 42;

    // IMU-only view constants
    static constexpr int IMU_TITLE_X = 44;
    static constexpr int IMU_TITLE_Y = 0;
    static constexpr int IMU_AXIS_X = 4;
    static constexpr int IMU_AXIS_Y = 16;
    static constexpr int IMU_ONLY_X = 30;
    static constexpr int IMU_ONLY_X_Y = 20;
    static constexpr int IMU_ONLY_Y_Y = 34;
    static constexpr int IMU_ONLY_Z_Y = 48;

    // Helper to print formatted text at a position
    void print_at_position(const char* text, int x, int y);
    bool is_view_active(View view) const;
    void show_active_view_locked();
    void draw_satellite_view_layout();
    void draw_temp_pressure_view_layout();
    void draw_imu_view_layout();
    void render_satellite_values();
    void render_temp_pressure_values();
    void render_imu_values();
    void draw_bitmap_1bpp(const uint8_t* bitmap, int width, int height, int x, int y);
    void draw_satellite_icon(int x, int y);
    void draw_clock_icon(int x, int y);
    void draw_imu_axis_icon(int x, int y);
    void draw_temp_pressure_icon(int x, int y);
    void draw_wifi_icon(int x, int y);
    void draw_waiting_icon(int x, int y);

    // Cached values used to repopulate a view when switching between layouts.
    static constexpr int TIME_TEXT_LEN = 6;
    char mtk_time_cache[TIME_TEXT_LEN] = "--:--";
    char ublox_time_cache[TIME_TEXT_LEN] = "--:--";
    int mtk_sat_cache = -1;
    int ublox_sat_cache = -1;
    double mtk_lat_cache = 0.0;
    double mtk_lon_cache = 0.0;
    double mtk_alt_cache = 0.0;
    bool has_mtk_location_cache = false;
    double ublox_lat_cache = 0.0;
    double ublox_lon_cache = 0.0;
    double ublox_alt_cache = 0.0;
    bool has_ublox_location_cache = false;

    int temp_val1_cache = 0;
    int temp_val2_cache = 0;
    int press_val1_cache = 0;
    int press_val2_cache = 0;
    bool has_temp_press_cache = false;

    double imu_x_cache = 0.0;
    double imu_y_cache = 0.0;
    double imu_z_cache = 0.0;
    bool has_imu_cache = false;
};

#endif  // OLED_LAYOUT_HPP
