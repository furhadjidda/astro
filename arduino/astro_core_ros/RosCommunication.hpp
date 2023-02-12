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

#ifndef ROS_COMMUNICATION_HPP_
#define ROS_COMMUNICATION_HPP_

// use this only if using MKR boards
#define USE_USBCON

#include <ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <Timer.h>
#include <MedianFilter.h>
#include "VL53L0X_core.hpp"
#include "DriveTrain.hpp"
#include "ImuCore.hpp"

class RosCommunication
{
public:
    RosCommunication();
    void InitNode();

    void PublishData();

    void SpinOnce()
    {
        mNodeHandle.spinOnce();
    }

    void CommandVelocityCallback(const geometry_msgs::Twist &aMsg)
    {
        mDriveTrain.CommandVelocityCallback(aMsg);
    }

private:
    void InitMotorCarrier();

    ros::NodeHandle mNodeHandle;
    VL53L0X mSensor_vl53l0x;
    ImuCore mSensorImu{mNodeHandle};
    DriveTrain mDriveTrain{mNodeHandle};

    // Odometry publisher
    nav_msgs::Odometry mOdom;
    ros::Publisher mOdomPublisher{"odom", &mOdom};

    // Imu publisher
    sensor_msgs::Imu mImuData;
    ros::Publisher mImuPub{"imu", &mImuData};

    // Ranger publisher
    sensor_msgs::Range mRangeData;
    ros::Publisher mRangePub{"irange", &mRangeData};

    // Teleop subscriber
    ros::Subscriber<geometry_msgs::Twist, RosCommunication> mTeleopSubscriber;
};
#endif
