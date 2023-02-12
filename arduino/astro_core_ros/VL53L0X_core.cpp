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

 #include "VL53L0X_core.hpp"
 #include <limits.h>

void VL53L0X::Init(ros::NodeHandle& aNh )
{
    // Node Handle Instance for logging
    mNodeHandle = aNh;

    if (!mSensor.begin()) 
    {
      //mNodeHandle.logwarn("Failed to setup VL53L0X sensor");
      while(1);
    }

    //mNodeHandle.loginfo("VL53L0X API serial node started");
    // fill static range message fields
    mRangeMsg.radiation_type = sensor_msgs::Range::INFRARED;
    mRangeMsg.header.frame_id =  "ir_ranger";
    mRangeMsg.field_of_view = 0.44; //25 degrees
    mRangeMsg.min_range = 0.03;
    mRangeMsg.max_range = 1.3;
} 

void VL53L0X::GetRangeData( sensor_msgs::Range& aRangeData )
{
    mSensor.rangingTest(&mMeasure, false);
    if (mMeasure.RangeStatus != 4)
    {  
        // phase failures have incorrect data
        mRangeMsg.range = (float)mMeasure.RangeMilliMeter/1000.0f; // convert mm to m
        mRangeMsg.header.stamp = mNodeHandle.now();
    }
    else
    {
      //mNodeHandle.logwarn("Out of range"); // if out of range, don't send message
      mRangeMsg.range = (float)1300/1000.0f;
      mRangeMsg.header.stamp = mNodeHandle.now();
    }
    aRangeData = mRangeMsg;
}