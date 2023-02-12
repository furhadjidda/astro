/*
 *   This file is part of ArduinoMkdir_SensorNode.
 *
 *    ArduinoMkdir_SensorNode is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   ArduinoMkdir_SensorNode is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with ArduinoMkdir_SensorNode.  If not, see <https://www.gnu.org/licenses/>.
 *   Description: ImuCore Header file
 *   Author: Furhad Jidda
 */
#ifndef VL53L0X_CORE_HPP
#define VL53L0X_CORE_HPP

//#define ROSSERIAL_ARDUINO_WIFI_MKR1010
//#define ARDUINO_MKR_WIFI

#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include "Adafruit_VL53L0X.h"

class VL53L0X{

  public:
      void Init(ros::NodeHandle& aNh );      

      void GetRangeData( sensor_msgs::Range& aRangeData );
  private:
      ros::NodeHandle mNodeHandle;
      VL53L0X_RangingMeasurementData_t mMeasure;
      sensor_msgs::Range mRangeMsg;
      Adafruit_VL53L0X mSensor{Adafruit_VL53L0X()};

};


#endif