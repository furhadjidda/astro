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

#include <sensor_msgs/msg/nav_sat_fix.h>
#include <sensor_msgs/msg/nav_sat_status.h>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
class gnss_parser{

    public:
        sensor_msgs__msg__NavSatFix parseGGA(const std::vector<std::string> &tokens);
        std::vector<std::string> split(const std::string &str, char delimiter);
        sensor_msgs__msg__NavSatFix packData(const double& latitude, const char lat, const double& longitude, const char lon, const double& altitude , const bool& fix);

};