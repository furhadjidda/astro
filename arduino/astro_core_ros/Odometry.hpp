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
#include <ros.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <cmath>
#include <ArduinoMotorCarrier.h>
#include "MotorControl.hpp"

#ifndef ODOMETRY_MOTOR_HPP
#define ODOMETRY_MOTOR_HPP

class Odometry
{
public:
    Odometry(ros::NodeHandle& nh) :
        mNodeHandle(nh),
        mWheelRadius(0.037),
        mWheelDistance(0.155),
        mEncoderResolution(900),
        mLastLeftEncoderCount(0),
        mLastRightEncoderCount(0),
        mLeftSpeed(0),
        mRightSpeed(0),
        mX(0),
        mY(0),
        mTheta(0),
        mLastTime(mNodeHandle.now()),
        mRate(10)
        //odom_pub_(mNodeHandle.advertise<nav_msgs::Odometry>("odom", 10)),
        //pid_left_speed_(mNodeHandle.param<double>("pid_left_speed_p", 1.0), mNodeHandle.param<double>("pid_left_speed_i", 0.0), mNodeHandle.param<double>("pid_left_speed_d", 0.0), mNodeHandle.param<double>("pid_left_speed_i_max", 1.0), mNodeHandle.param<double>("pid_left_speed_i_min", -1.0)),
        //pid_right_speed_(mNodeHandle.param<double>("pid_right_speed_p", 1.0), mNodeHandle.param<double>("pid_right_speed_i", 0.0), mNodeHandle.param<double>("pid_right_speed_d", 0.0), mNodeHandle.param<double>("pid_right_speed_i_max", 1.0), mNodeHandle.param<double>("pid_right_speed_i_min", -1.0))
    {
    }
        
    void InitNode();

    void CommandVelocityCallback(const geometry_msgs::Twist& msg);

    void UpdateOdometry();

    void GetOdomData(  nav_msgs::Odometry& aOdom );

private:
    double mWheelRadius;
    double mWheelDistance;
    int mEncoderResolution;
    int mLastLeftEncoderCount;
    int mLastRightEncoderCount;
    double mLeftSpeed;
    double mRightSpeed;
    double mX;
    double mY;
    double mTheta;
    ros::Time mLastTime;
    int mRate;
    ros::NodeHandle mNodeHandle;
    //ros::Publisher odom_pub_;
    //pid::PID pid_left_speed_;
    //pid::PID pid_right_speed_;
    MotorControl* mRightMotor{nullptr};
    MotorControl* mLeftMotor{nullptr};
};



#endif // ODOMETRY_HPP