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

#include "WiFiHandler.hpp"
#include <utility/wifi_drv.h>

char ssid[] = "JiddaHouse";        // your network SSID (name)
char pass[] = "Jidda17536";    // your network password (use for WPA, or use as key for WEP)

void WifiHandler::SetupWifi()
{
    // check for the WiFi module:
    if (WiFi.status() == WL_NO_MODULE) {
      Serial.println("Communication with WiFi module failed!");
      // don't continue
      while (true);
    }

    String fv = WiFi.firmwareVersion();
    if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
      Serial.println("Please upgrade the firmware");
    }
    Serial.print("Firmware version = ");
    Serial.println(fv);

    // attempt to connect to WiFi network:
    while (status != WL_CONNECTED) {
      Serial.print("Attempting to connect to WPA SSID: ");
      Serial.println(ssid);
      // Connect to WPA/WPA2 network:
      status = WiFi.begin(ssid, pass);
    }

    // you're connected now, so print out the data:
    Serial.print("You're connected to the network");
    PrintCurrentNet();
    PrintWifiData();
    delay(500);
}


void WifiHandler::GetServerAndPort( uint16_t& aPort, IPAddress& aServer )
{
  aPort = mPort;
  aServer = mIPAddress;
}

void WifiHandler::PrintWifiData() 
{
    // print your board's IP address:
    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);

    // print your MAC address:
    byte mac[6];
    WiFi.macAddress(mac);
    Serial.print("MAC address: ");
    PrintMacAddress(mac);
}

void WifiHandler::PrintCurrentNet() 
{
    // print the SSID of the network you're attached to:
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    // print the MAC address of the router you're attached to:
    byte bssid[6];
    WiFi.BSSID(bssid);
    Serial.print("BSSID: ");
    PrintMacAddress(bssid);

    // print the received signal strength:
    long rssi = WiFi.RSSI();
    Serial.print("signal strength (RSSI):");
    Serial.println(rssi);

    // print the encryption type:
    byte encryption = WiFi.encryptionType();
    Serial.print("Encryption Type:");
    Serial.println(encryption, HEX);
    Serial.println();
}

void WifiHandler::PrintMacAddress(byte mac[]) 
{
    for (int i = 5; i >= 0; i--) {
      if (mac[i] < 16) {
        Serial.print("0");
      }
      Serial.print(mac[i], HEX);
      if (i > 0) {
        Serial.print(":");
      }
    }
    Serial.println();
}
