/*
 *   This file is part of astro_core_ros.
 *
 *   astro_core_ros is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   astro_core_ros is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License v3.0
 *   along with astro_core_ros.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef WIFI_HANDLER_HPP
#define WIFI_HANDLER_HPP

#include <SPI.h>
#include <WiFiNINA.h>

class WifiHandler{
    public:
        void SetupWifi();
        void GetServerAndPort( uint16_t& aPort, IPAddress& aServer );
        void PrintWifiData();
        void PrintStatus()
        {
            long rssi = WiFi.RSSI();
            if( rssi < -60 )
            {
                Serial.print("signal strength (RSSI):");
                Serial.println(rssi);
            }

            if( WiFi.status() != WL_CONNECTED )
            {
                Serial.println("Something wrong with connection");
            }
        }
    private:
        uint16_t mPort{11411};
        IPAddress mIPAddress{192,168,1,212};
        int status{WL_IDLE_STATUS};     // the WiFi radio's status
        
        void PrintCurrentNet();
        void PrintMacAddress(byte mac[]);

};

#endif