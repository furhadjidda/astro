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

#include "RosCommunication.hpp"

RosCommunication::RosCommunication()
    : mTeleopSubscriber("cmd_vel", &RosCommunication::CommandVelocityCallback, this)
{
}

void RosCommunication::InitNode()
{
    // Set the ros serial at 115200
    mNodeHandle.getHardware()->setBaud(115200);
    // Initialize the ros node
    mNodeHandle.initNode();
    Serial.begin(115200);
    // Advertise whats being published
    mNodeHandle.advertise(mOdomPublisher);
    mNodeHandle.advertise(mImuPub);
    mNodeHandle.advertise(mRangePub);
    mNodeHandle.subscribe(mTeleopSubscriber);
    // Initialize hardware
    // Initialize motors and encoders
    mDriveTrain.InitNode();
    // Initialize range sensor
    mSensor_vl53l0x.Init(mNodeHandle);
    // Initialize IMU
    mSensorImu.Init();
}

void RosCommunication::PublishData()
{
    mDriveTrain.UpdateOdometry();
    mDriveTrain.GetOdomData(mOdom);
    mDriveTrain.Ping();
    mOdom.header.stamp = mNodeHandle.now();
    mOdom.header.frame_id = "odom";
    mOdom.child_frame_id = "base_link";
    mOdomPublisher.publish(&mOdom);

    mSensorImu.GetIMUData(mImuData);
    mImuData.header.stamp = mNodeHandle.now();
    mImuData.header.frame_id = "imu_link";
    mImuPub.publish(&mImuData);

    mSensor_vl53l0x.GetRangeData(mRangeData);
    mRangeData.header.stamp = mNodeHandle.now();
    mRangePub.publish(&mRangeData);
}