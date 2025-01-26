/*
 *   This file is part of astro project.
 *
 *   astro projec is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   astro project is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License v3.0
 *   along with astro project.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "gnss_parser.hpp"

sensor_msgs__msg__NavSatFix gnss_parser::parseGGA(const std::vector<std::string> &tokens) {
    sensor_msgs__msg__NavSatFix msg;

    // Check if there are enough tokens
    if (tokens.size() < 15) {
        // std::cerr << "Invalid GGA sentence: insufficient tokens" << std::endl;
        return msg; // Return an empty message if the sentence is invalid
    }

    // Latitude
    double lat = 0.0;
    lat = std::stod(tokens[2]); // Parse the latitude
    if (lat == 0.0) {
        return msg;
    }

    double lat_deg = static_cast<int>(lat / 100);
    double lat_min = lat - lat_deg * 100;
    msg.latitude = lat_deg + lat_min / 60.0;
    if (tokens[3] == "S") {
        msg.latitude = -msg.latitude;
    }

    // Longitude
    double lon = 0.0;
    lon = std::stod(tokens[4]); // Parse the longitude
    if (lon == 0.0) {
        return msg;
    }

    double lon_deg = static_cast<int>(lon / 100);
    double lon_min = lon - lon_deg * 100;
    msg.longitude = lon_deg + lon_min / 60.0;
    if (tokens[5] == "W") {
        msg.longitude = -msg.longitude;
    }

    // Altitude
    msg.altitude = std::stod(tokens[9]);
    if (msg.altitude == 0.0) {
        return msg;
    }

    // Status
    int status = 0;
    status = std::stoi(tokens[6]); // Convert the status to integer
    if (status == 0) {
        return msg;
    }

    // Set the status based on the value
    msg.status.status = (status == 1 || status == 2) ? sensor_msgs__msg__NavSatStatus__STATUS_FIX
                                                     : sensor_msgs__msg__NavSatStatus__STATUS_NO_FIX;
    msg.status.service = sensor_msgs__msg__NavSatStatus__SERVICE_GPS;

    return msg;
}

sensor_msgs__msg__NavSatFix gnss_parser::packData(const double &latitude, const char lat, const double &longitude,
                                                  const char lon, const double &altitude, const bool &fix,
                                                  const uint8_t &fixQuality, const double &hdop, const double &pdop,
                                                  const double &vdop) {
    sensor_msgs__msg__NavSatFix msg = {};
    msg.header.frame_id.data = "gnss_frame";
    msg.header.frame_id.size = sizeof("gnss_frame");
    // Latitude
    double lat_deg = static_cast<int>(latitude / 100);
    double lat_min = latitude - lat_deg * 100;
    msg.latitude = lat_deg + lat_min / 60.0;
    if (lat == 'S') {
        msg.latitude = -msg.latitude;
    }

    // Longitude
    double lon_deg = static_cast<int>(longitude / 100);
    double lon_min = longitude - lon_deg * 100;
    msg.longitude = lon_deg + lon_min / 60.0;
    if (lon == 'W') {
        msg.longitude = -msg.longitude;
    }

    // Altitude
    msg.altitude = altitude;
    // Status
    int status = fix;
    // Set the status based on the value
    msg.status.status =
        (status == true) ? sensor_msgs__msg__NavSatStatus__STATUS_FIX : sensor_msgs__msg__NavSatStatus__STATUS_NO_FIX;
    msg.status.service = sensor_msgs__msg__NavSatStatus__SERVICE_GPS;

    msg.position_covariance_type = fixQuality;

    // position covariance
    msg.position_covariance[0] = hdop * hdop;
    msg.position_covariance[4] = hdop * hdop;
    msg.position_covariance[8] = vdop * vdop;

    return msg;
}

std::vector<std::string> gnss_parser::split(const std::string &str, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(str);
    while (std::getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}
