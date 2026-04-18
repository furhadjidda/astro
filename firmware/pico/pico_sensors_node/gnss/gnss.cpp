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

#include "gnss.hpp"

#include <rmw_microros/rmw_microros.h>

#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#define GPS_I2C_ADDRESS 0x10

bool GNSSSensor::initialize(i2c_inst_t *aI2cInstance) {
    if (!GPS->Init(GPS_I2C_ADDRESS)) {
        return false;
    }
    // Configure GPS
    GPS->SendCommand(reinterpret_cast<const uint8_t *>(PMTK_SET_NMEA_OUTPUT_ALLDATA),
                     strlen(PMTK_SET_NMEA_OUTPUT_ALLDATA));
    GPS->SendCommand(reinterpret_cast<const uint8_t *>(PMTK_SET_NMEA_UPDATE_1HZ), strlen(PMTK_SET_NMEA_UPDATE_1HZ));
    GPS->SendCommand(reinterpret_cast<const uint8_t *>(PGCMD_ANTENNA), strlen(PGCMD_ANTENNA));
    sleep_ms(1000);
    return true;
}

void GNSSSensor::get_debug_message(std_msgs__msg__String &msg) {
    rosidl_runtime_c__String__assign(&msg.data, debug_msg_buffer);
}

void GNSSSensor::get_gnss_data(sensor_msgs__msg__NavSatFix &msg) {
    char c = GPS->ReadData();

    if (GPS->NewNMEAreceived()) {
        std::string nmea_sentence(GPS->LastNMEA());
        if (!GPS->Parse(GPS->LastNMEA())) {
            return;
        }
        // Publish debug message
        std::string fix_status = (GPS->mFix) ? "true" : "false";
        // Pack and publish GNSS data
        msg = parser->packData(GPS->mLatitude, GPS->mLat, GPS->mLongitude, GPS->mLon, GPS->mAltitude, GPS->mFix,
                               GPS->mFixquality_3d, GPS->mHDOP, GPS->mPDOP, GPS->mVDOP);

        rcl_time_point_value_t now = rmw_uros_epoch_nanos();
        msg.header.stamp.sec = now / 1e9;
        msg.header.stamp.nanosec = now % (uint32_t)1e9;
    }
}