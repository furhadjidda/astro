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

#include "odometry.hpp"
#include "common.hpp"
#include <chrono>
#include <geometry_msgs/msg/transform_stamped.h>
#include <micro_ros_utilities/string_utilities.h>
#include <micro_ros_utilities/type_utilities.h>
#include <tf2_msgs/msg/tf_message.h>

const int timeout_ms = 1000;

template <typename type> type sign(type value) { return type((value > 0) - (value < 0)); }

void Odometry::Init(motor_control *aMotorControl) { mMotorControl = aMotorControl; }

void Odometry::SetPublisher(rcl_publisher_t *aPub) { mLogPublisher = aPub; }

void Odometry::UpdateOdometry() {
    mCurrentTime = millis();
    unsigned long elapsedTime = mCurrentTime - mPreviousTime;
    mPreviousTime = mCurrentTime;

    // Publish Debug data on debug topic
    memset(mLogBuffer, 0x00, sizeof(mLogBuffer));
    std_msgs__msg__String debugMsg;

    // Process odometry updates every FREQUENCY_RATE milliseconds
    if (FREQUENCY_RATE <= elapsedTime) {
        UpdateMotorOdometry(MotorId::MotorA, mMotorRight_PrevTime, mMotorRight_FilteredIncrementPerSecond,
                            mMotorRight_RateEst, mMotorRight_FilteredDirection, mMotorRight_DirMedianFilter);

        UpdateMotorOdometry(MotorId::MotorB, mMotorLeft_PrevTime, mMotorLeft_FilteredIncrementPerSecond,
                            mMotorLeft_RateEst, mMotorLeft_FilteredDirection, mMotorLeft_DirMedianFilter);
    }

    // Control motor rates every FREQUENCY_CONTROLLER milliseconds
    if (FREQUENCY_CONTROLLER <= elapsedTime) {
        ControlMotorRate(MotorId::MotorA, mMotorRight_RateRef, mMotorRight_RateEst, mMotorRight_PwmRate, mMotorRightPwm,
                         mController_MotorRightPrevTime, mController_MotorRightPrevEpsilon, mController_MotorRightInt);

        ControlMotorRate(MotorId::MotorB, mMotorLeft_RateRef, mMotorLeft_RateEst, mMotorLeft_PwmRate, mMotorLeftPwm,
                         mController_MotorLeftPrevTime, mController_MotorLeftPrevEpsilon, mController_MotorLeftInt);

        PublishDebugData(elapsedTime);
    }
}

// Helper function to update motor odometry
void Odometry::UpdateMotorOdometry(MotorId motorId, unsigned long &motorPrevTime,
                                   float &motorFilteredIncrementPerSecond, float &motorRateEst,
                                   int &motorFilteredDirection, median_filter &motorDirFilter) {
    float dt = (millis() - motorPrevTime);
    motorPrevTime = millis();

    // Update motor direction using median filter
    motorDirFilter.in(mMotorControl->GetMotorDirection(motorId));
    motorFilteredDirection = motorDirFilter.out();

    // Calculate motor increment per second
    motorFilteredIncrementPerSecond = RunningAverage(
        motorFilteredIncrementPerSecond,
        static_cast<float>(mMotorControl->GetEncoderIncrementValue(motorId)) / dt * 1000.0f, RATE_AVERAGE_FILTER_SIZE);

    // Estimate motor rate
    motorRateEst = static_cast<float>(motorFilteredDirection) * motorFilteredIncrementPerSecond * RATE_CONV;

    // Reset encoder increment value
    mMotorControl->ResetEncoderIncrementValue(motorId);

    // Apply rate threshold to prevent noise
    if (abs(motorRateEst) < 0.1f) {
        motorRateEst = 0.0f;
    }
}

// Helper function to control motor rate using a PID controller
void Odometry::ControlMotorRate(MotorId motorId, float motorRateRef, float motorRateEst, int &motorPwmRate,
                                int &motorPwm, unsigned long &controllerPrevTime, float &controllerPrevEpsilon,
                                float &controllerInt) {
    RateControler(motorRateRef, motorRateEst, motorPwmRate, controllerPrevTime, controllerPrevEpsilon, controllerInt);
    motorPwm += motorPwmRate;
    motorPwm = constrain(static_cast<int>(motorPwm), 0, PWM_MAX);
    mMotorControl->SetMotorRateAndDirection(motorId, motorPwm, motorRateRef);
}

// Helper function to publish debug data
void Odometry::PublishDebugData(unsigned long elapsedTime) {
    std::string json_message = "{\"LEFT :: mMotorLeftPwm\":" + std::to_string(abs(mMotorLeftPwm)) +
                               ",\"LEFT :: mMotorLeft_RateRef\":\"" + std::to_string(mMotorLeft_RateRef) +
                               ",\"RIGHT :: mMotorRightPwm\":\"" + std::to_string(abs(mMotorRightPwm)) +
                               ",\"RIGHT :: mMotorRight_RateRef\":\"" + std::to_string(mMotorRight_RateRef) +
                               "\",\"elapsedTime\":\"" + std::to_string(elapsedTime) + "\"}";

    std_msgs__msg__String debugMsg;
    if (!rosidl_runtime_c__String__init(&debugMsg.data)) {
        // Handle initialization failure (e.g., log an error)
        return;
    }

    // Assign the std::string value to the ROS message
    rosidl_runtime_c__String__assign(&debugMsg.data, json_message.c_str());

    // Publish the message
    rcl_publish(mLogPublisher, &debugMsg, NULL);

    // Clean up allocated memory
    rosidl_runtime_c__String__fini(&debugMsg.data);
}

void Odometry::CalculateOdometry(nav_msgs__msg__Odometry &aOdometry) {
    float dt, dx, dy;
    float qw, qx, qy, qz;

    dt = (float)(millis() - mOdomPrevTime) * 0.001f;
    mOdomPrevTime = millis();

    // compute linear and angular estimated velocity
    // Refer to the equations here - http://planning.cs.uiuc.edu/node659.html
    mLinearVelocityEst = WHEEL_RADIUS * (mMotorRight_RateEst + mMotorLeft_RateEst) / 2.0f;
    mAngularVelocityEst = (WHEEL_RADIUS / BASE_LENGTH) * (mMotorRight_RateEst - mMotorLeft_RateEst);

    // compute translation and rotation
    mYawEst += mAngularVelocityEst * dt;
    dx = cos(mYawEst) * mLinearVelocityEst * dt;
    dy = sin(mYawEst) * mLinearVelocityEst * dt;

    // compute quaternion
    qw = cos(abs(mYawEst) / 2.0f);
    qx = 0.0f;
    qy = 0.0f;
    qz = sign(mYawEst) * sin(abs(mYawEst) / 2.0f);

    // feed odom message
    // aOdometry.header.stamp.nanosec = rmw_uros_epoch_nanos();
    aOdometry.header.frame_id = mFrameId;
    // aOdometry.child_frame_id = mChildFrameId;
    aOdometry.pose.pose.position.x += dx;
    aOdometry.pose.pose.position.y += dy;
    aOdometry.pose.pose.position.z = 0.0;
    aOdometry.pose.pose.orientation.w = qw;
    aOdometry.pose.pose.orientation.x = qx;
    aOdometry.pose.pose.orientation.y = qy;
    aOdometry.pose.pose.orientation.z = qz;
    // Velocity expressed in base_link frame
    aOdometry.twist.twist.linear.x = mLinearVelocityEst;
    aOdometry.twist.twist.linear.y = 0.0f;
    aOdometry.twist.twist.angular.z = mAngularVelocityEst;
}

void Odometry::cmd_vel_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    mLinearVelocityRef = msg->linear.x;
    mAngularVelocityRef = msg->angular.z;

    // MIXER
    mMotorRight_RateRef = (mLinearVelocityRef + BASE_LENGTH / 2.f * mAngularVelocityRef) / (WHEEL_RADIUS);

    mMotorLeft_RateRef = (mLinearVelocityRef - BASE_LENGTH / 2.f * mAngularVelocityRef) / (WHEEL_RADIUS);

    // logging
    memset(mLogBuffer, 0x00, sizeof(mLogBuffer));
    std_msgs__msg__String debugMsg;
    std::string json_message = "{\"mMotorLeftPwm\":" + std::to_string(mMotorLeftPwm) + ",\"mMotorLeft_RateRef\":\"" +
                               std::to_string(mMotorLeft_RateRef) + ",\"mMotorRightPwm\":\"" +
                               std::to_string(mMotorLeftPwm) + ",\"mMotorRight_RateRef\":\"" +
                               std::to_string(mMotorRight_RateRef) + "\"}";
    if (!rosidl_runtime_c__String__init(&debugMsg.data)) {
        // Handle initialization failure (e.g., log an error)
        return;
    }

    // Assign the std::string value to the ROS message
    rosidl_runtime_c__String__assign(&debugMsg.data, json_message.c_str());

    // Publish the message
    rcl_publish(mLogPublisher, &debugMsg, NULL);

    // Clean up allocated memory
    rosidl_runtime_c__String__fini(&debugMsg.data);
}

void Odometry::RateControler(const float aRateRef, const float aRateEst, int &aPwmRate, unsigned long &aPrevTime,
                             float &aPrevEpsilon, float &aIEpsilon) {
    float epsilon = abs(aRateRef) - abs(aRateEst);
    float d_epsilon = (epsilon - aPrevEpsilon) / (aPrevTime - millis());

    // reset and clamp integral (todo : add anti windup)
    if (aRateRef == 0.0) {
        aIEpsilon = 0.0;
    } else {
        aIEpsilon += epsilon * (aPrevTime - millis()) * RATE_CONTROLLER_KI;
    }
    aIEpsilon =
        constrain(aIEpsilon, static_cast<float>(-RATE_INTEGRAL_FREEZE), static_cast<float>(RATE_INTEGRAL_FREEZE));

    aPrevTime = millis();
    aPrevEpsilon = epsilon;

    aPwmRate = epsilon * RATE_CONTROLLER_KP + d_epsilon * RATE_CONTROLLER_KD + aIEpsilon * RATE_CONTROLLER_KI;

    // saturate output
    aPwmRate = constrain(aPwmRate, RATE_CONTROLLER_MIN_PWM, RATE_CONTROLLER_MAX_PWM);
}

float Odometry::RunningAverage(float mPrevAvg, const float aVal, const int aSize) {
    return (mPrevAvg * (aSize - 1) + aVal) / aSize;
}
