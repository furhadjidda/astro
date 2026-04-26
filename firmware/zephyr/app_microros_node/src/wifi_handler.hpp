#pragma once

#include <zephyr/kernel.h>

class OLEDLayout;

class WifiHandler {
   public:
    int initStation(OLEDLayout& oled_layout);

   private:
    int connectWithRetry();
    int tryConnect();
};
