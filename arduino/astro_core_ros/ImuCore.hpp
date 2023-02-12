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

#ifndef IMUCORE_HPP
#define IMUCORE_HPP

//#define ROSSERIAL_ARDUINO_WIFI_MKR1010
//#define ARDUINO_MKR_WIFI

#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

class ImuCore{
    public:
        ImuCore(ros::NodeHandle& aNh):mNodeHandle(aNh)
        {}
        virtual ~ImuCore(){}
        void Init();

        void GetIMUData( sensor_msgs::Imu& aImuData );

    private:
        ros::NodeHandle& mNodeHandle;
        geometry_msgs::Quaternion  mQuaternion;        // [w, x, y, z]         quaternion container
        float mAccel[3];             // [x, y, z]            accel sensor measurements
        //VectorFloat mGravity;           // [x, y, z]            gravity vector
        float mEuler[3];                 // [psi, theta, phi]    Euler angle container
        float mYpr[3];                   // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
        int32_t mGyro[3];                  //Angular velocity
        Adafruit_BNO055 mBno055{Adafruit_BNO055(55, 0x28)};
    
};

#endif // IMUCORE_HPP
