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

#ifndef __ODOMETRY_HPP__
#define __ODOMETRY_HPP__
#include "common.hpp"
#include "encoder.hpp"
#include "median_filter.hpp"
#include "motor_control.hpp"
#include <cmath>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include <sys/_stdint.h>

class Odometry {

  public:
    void Init(motor_control *aMotorControl);
    void SetPublisher(rcl_publisher_t *aPub);
    void CalculateOdometry(nav_msgs__msg__Odometry &aOdometry);
    void UpdateOdometry();
    void cmd_vel_callback(const void *msgin);

  private:
    float RunningAverage(float mPrevAvg, const float aVal, const int aSize);
    void UpdateMotorOdometry(MotorId motorId, unsigned long &motorPrevTime, float &motorFilteredIncrementPerSecond,
                             float &motorRateEst, int &motorFilteredDirection, median_filter &motorDirFilter);
    void ControlMotorRate(MotorId motorId, float motorRateRef, float motorRateEst, int &motorPwmRate, int &motorPwm,
                          unsigned long &controllerPrevTime, float &controllerPrevEpsilon, float &controllerInt);
    void PublishDebugData(unsigned long elapsedTime);
    void RateControler(const float aRateRef, const float aRateEst, int &aPwmRate, unsigned long &aPrevTime,
                       float &aPrevEpsilon, float &aIEpsilon);

    motor_control *mMotorControl;
    geometry_msgs__msg__Quaternion mQuat;
    rosidl_runtime_c__String mFrameId = {"odom", strlen("odom"), sizeof("odom")};
    rosidl_runtime_c__String mChildFrameId = {"base_footprint", strlen("base_footprint"), sizeof("base_footprint")};
    rcl_publisher_t *mLogPublisher;
    unsigned long mCurrentTime;
    unsigned long mPreviousTime;
    int mLastLeftEncoderCount;
    int mLastRightEncoderCount;

    // Logging
    // StaticJsonDocument<500> mJsonDoc;
    uint8_t mLogBuffer[500];

    /* Define median filter for direction */
    // https://en.wikipedia.org/wiki/Median_filter#Worked_one-dimensional_example
    // The median filter is a non-linear digital filtering technique, often used
    // to remove noise from an image or signal.
    median_filter mMotorRight_DirMedianFilter{RATE_DIRECTION_MEDIAN_FILTER_SIZE};
    median_filter mMotorLeft_DirMedianFilter{RATE_DIRECTION_MEDIAN_FILTER_SIZE};

    /* Mixer variable */
    float mLinearVelocityRef;
    float mAngularVelocityRef;

    /* Mixer variable */
    float mLinearVelocityEst;
    float mAngularVelocityEst;

    // right
    int mMotorRight_FilteredDirection;
    float mMotorRight_FilteredIncrementPerSecond;
    float mMotorRight_RateEst;
    float mMotorRight_RateRef;
    int mMotorRight_PwmRate;
    unsigned long mMotorRight_PrevTime;
    int mMotorRightPwm = 0;

    // left
    int mMotorLeft_FilteredDirection;
    float mMotorLeft_FilteredIncrementPerSecond;
    float mMotorLeft_RateEst;
    float mMotorLeft_RateRef;
    int mMotorLeft_PwmRate;
    unsigned long mMotorLeft_PrevTime;
    int mMotorLeftPwm = 0;

    /* Define controllers variables */
    // right
    unsigned long mController_MotorRightPrevTime;
    float mController_MotorRightPrevEpsilon = 0.0;
    float mController_MotorRightInt = 0.0;
    // left
    unsigned long mController_MotorLeftPrevTime;
    float mController_MotorLeftPrevEpsilon = 0.0;
    float mController_MotorLeftInt = 0.0;

    float mYawEst;
    unsigned long mOdomPrevTime;
};

#endif