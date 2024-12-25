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

#include <gtest/gtest.h>
#include "gnss_parser.hpp"

// Test fixture for Adafruit_GPS
class GnssParserTests : public ::testing::Test {
   protected:
    // Test fixture constructor
    GnssParserTests() {}  // Initialize Adafruit_GPS with mock arguments

    void SetUp() override {
        // Initialize any necessary objects or states here
    }

    void TearDown() override {
        // Clean up any objects or states here
    }
};

TEST_F(GnssParserTests, ParseValidGGA) {
    std::string nmea_sentence("$GNGGA,050727.000,3850.2311,N,09447.3415,W,1,7,1.16,342.7,M,-30.0,M,,*78");
    gnss_parser parser;
    ASSERT_NE(nmea_sentence.find("$GNGGA"), std::string::npos);
    ASSERT_EQ((std::string::npos != nmea_sentence.find("$GPGGA") || std::string::npos != nmea_sentence.find("$GNGGA")), true);
    auto tokens = parser.split(nmea_sentence,',');
    ASSERT_GE(tokens.size(),15);
    parser.parseGGA(tokens);
}
// $GNGGA,235943.100,,,,,0,0,,,M,,M,,*5D
TEST_F(GnssParserTests, ParseValidGGA2) {
    std::string nmea_sentence("$GNGGA,235943.100,,,,,0,0,,,M,,M,,*5D");
    gnss_parser parser;
    ASSERT_NE(nmea_sentence.find("$GNGGA"), std::string::npos);
    ASSERT_EQ((std::string::npos != nmea_sentence.find("$GPGGA") || std::string::npos != nmea_sentence.find("$GNGGA")), true);
    auto tokens = parser.split(nmea_sentence,',');
    ASSERT_GE(tokens.size(),15);
    for (auto& token : tokens) {
        if (token.empty()) {
            token = "0";
        }
    }

    parser.parseGGA(tokens);
}

// $GNGGA,000005.875,,,,,0,0,,,M,,M,,*59
// @todo This test fails because parseGGA throws and exception for now and needs to be worked on.
TEST_F(GnssParserTests, ParseValidGGA3) {
    std::string nmea_sentence("$GNVTG,0.00,T,,M,0.00,N,0.00,K,N*2C\r$GNGGA,000009.100,,,,,0,0,,,M,,M,,*5E\r\n");
    gnss_parser parser;
    ASSERT_NE(nmea_sentence.find("$GNGGA"), std::string::npos);
    ASSERT_EQ((std::string::npos != nmea_sentence.find("$GPGGA") || std::string::npos != nmea_sentence.find("$GNGGA")), true);
    auto tokens = parser.split(nmea_sentence,',');
    ASSERT_GE(tokens.size(),15);
    for (auto& token : tokens) {
        if (token.empty()) {
            token = "0";
        }
    }

    parser.parseGGA(tokens);
}
// Add more tests as needed
