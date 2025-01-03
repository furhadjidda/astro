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

#include "Odometry.hpp"
#include "Common.hpp"
#include <chrono>
#include <geometry_msgs/msg/transform_stamped.h>
#include <micro_ros_utilities/string_utilities.h>
#include <micro_ros_utilities/type_utilities.h>
#include <tf2_msgs/msg/tf_message.h>

const int timeout_ms = 1000;

template <typename type> type sign(type value) {
  return type((value > 0) - (value < 0));
}

void Odometry::Init(MotorControl *aMotorControl) {
  mMotorControl = aMotorControl;
}

void Odometry::SetPublisher(rcl_publisher_t *aPub) { mLogPublisher = aPub; }

void Odometry::UpdateOdometry() {
  mCurrentTime = millis();
  unsigned long elapsedTime = mCurrentTime - mPreviousTime;
  mPreviousTime = mCurrentTime;

  /// Publish Debug data on debug topic
  memset(mLogBuffer, 0x00, sizeof(mLogBuffer));
  std_msgs__msg__String debugMsg;

  // Refer : http://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf
  if (FREQUENCY_RATE <= elapsedTime) {
    float dt;

    // MOTOR RIGHT
    // direction
    // Pass it through median filter to remove noise
    mMotorRight_DirMedianFilter.in(
        mMotorControl->GetMotorDirection(MotorId::MotorA));
    mMotorRight_FilteredDirection = mMotorRight_DirMedianFilter.out();

    // filter increment per second
    dt = (millis() - mMotorRight_PrevTime);
    mMotorRight_PrevTime = millis();
    // Calculate Motor Encoder Increment Value / per second
    mMotorRight_FilteredIncrementPerSecond = RunningAverage(
        mMotorRight_FilteredIncrementPerSecond,
        (float)mMotorControl->GetEncoderIncrementValue(MotorId::MotorA) / dt *
            1000.0f,
        RATE_AVERAGE_FILTER_SIZE);

    // Estimating Rate
    mMotorRight_RateEst = (float)mMotorRight_FilteredDirection *
                          mMotorRight_FilteredIncrementPerSecond * RATE_CONV;

    // Clear encoder increments
    mMotorControl->ResetEncoderIncrementValue(MotorId::MotorA);

    if (abs(mMotorRight_RateEst) < 0.1f) {
      mMotorRight_RateEst = 0.0f;
    }

    // mRightMotor.SetMotorCheckDirectionVal( 1 );
    // mRightMotor.SetMotorDirectionVal( 0 );

    // MOTOR LEFT
    // direction
    mMotorLeft_DirMedianFilter.in(
        mMotorControl->GetMotorDirection(MotorId::MotorB));
    mMotorLeft_FilteredDirection = mMotorLeft_DirMedianFilter.out();

    // filter increment per second
    dt = (millis() - mMotorLeft_PrevTime);
    mMotorLeft_PrevTime = millis();

    mMotorLeft_FilteredIncrementPerSecond = RunningAverage(
        mMotorLeft_FilteredIncrementPerSecond,
        (float)mMotorControl->GetEncoderIncrementValue(MotorId::MotorB) / dt *
            1000.0f,
        RATE_AVERAGE_FILTER_SIZE);

    // estimated rate
    mMotorLeft_RateEst = (float)mMotorLeft_FilteredDirection *
                         mMotorLeft_FilteredIncrementPerSecond * RATE_CONV;
    mMotorControl->ResetEncoderIncrementValue(MotorId::MotorB);

    if (abs(mMotorLeft_RateEst) < 0.1f) {
      mMotorLeft_RateEst = 0.0f;
    }
    // mLeftMotor.SetMotorCheckDirectionVal( 1 );
    // mLeftMotor.SetMotorDirectionVal( 0 );
  }

  if (FREQUENCY_CONTROLLER <= elapsedTime) {
    // MOTOR RIGHT
    RateControler(mMotorRight_RateRef, mMotorRight_RateEst, mMotorRight_PwmRate,
                  mController_MotorRightPrevTime,
                  mController_MotorRightPrevEpsilon, mController_MotorRightInt);
    mMotorRightPwm = mMotorRightPwm + mMotorRight_PwmRate;
    mMotorRightPwm = constrain(mMotorRightPwm, 0, PWM_MAX);
    mMotorControl->SetMotorRateAndDirection(MotorId::MotorA, mMotorRightPwm,
                                            mMotorRight_RateRef);

    // MOTOR LEFT
    RateControler(mMotorLeft_RateRef, mMotorLeft_RateEst, mMotorLeft_PwmRate,
                  mController_MotorLeftPrevTime,
                  mController_MotorLeftPrevEpsilon, mController_MotorLeftInt);
    mMotorLeftPwm = mMotorLeftPwm + mMotorLeft_PwmRate;
    mMotorLeftPwm = constrain(mMotorLeftPwm, 0, PWM_MAX);
    mMotorControl->SetMotorRateAndDirection(MotorId::MotorB, mMotorLeftPwm,
                                            mMotorLeft_RateRef);

    mJsonDoc["mMotorLeftPwm"] = mMotorLeftPwm;
    mJsonDoc["mMotorLeft_RateRef"] = mMotorLeft_RateRef;
    mJsonDoc["mMotorRightPwm"] = mMotorRightPwm;
    mJsonDoc["mMotorRight_RateRef"] = mMotorRight_RateRef;
    mJsonDoc["elapsedTime"] = elapsedTime;
    serializeJson(mJsonDoc, &mLogBuffer, 200);
    debugMsg.data.data = (char *)mLogBuffer;
    debugMsg.data.capacity = strlen((const char *)mLogBuffer);
    debugMsg.data.size = sizeof(mLogBuffer);
    rcl_publish(mLogPublisher, &debugMsg, NULL);
  }
}

void Odometry::CalculateOdometry(nav_msgs__msg__Odometry &aOdometry) {
  float dt, dx, dy;
  float qw, qx, qy, qz;

  dt = (float)(millis() - mOdomPrevTime) * 0.001f;
  mOdomPrevTime = millis();

  // compute linear and angular estimated velocity
  // Refer to the equations here - http://planning.cs.uiuc.edu/node659.html
  mLinearVelocityEst =
      WHEEL_RADIUS * (mMotorRight_RateEst + mMotorLeft_RateEst) / 2.0f;
  mAngularVelocityEst =
      (WHEEL_RADIUS / BASE_LENGTH) * (mMotorRight_RateEst - mMotorLeft_RateEst);

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
  aOdometry.child_frame_id = mChildFrameId;
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
  const geometry_msgs__msg__Twist *msg =
      (const geometry_msgs__msg__Twist *)msgin;
  mLinearVelocityRef = msg->linear.x;
  mAngularVelocityRef = msg->angular.z;

  // MIXER
  mMotorRight_RateRef =
      (mLinearVelocityRef + BASE_LENGTH / 2.f * mAngularVelocityRef) /
      (WHEEL_RADIUS);

  mMotorLeft_RateRef =
      (mLinearVelocityRef - BASE_LENGTH / 2.f * mAngularVelocityRef) /
      (WHEEL_RADIUS);

  // logging
  memset(mLogBuffer, 0x00, sizeof(mLogBuffer));
  std_msgs__msg__String debugMsg;
  mJsonDoc["linear"] = mLinearVelocityRef;
  mJsonDoc["angular"] = mAngularVelocityRef;
  mJsonDoc["mMotorRight_RateRef"] = mMotorRight_RateRef;
  mJsonDoc["mMotorLeft_RateRef"] = mMotorLeft_RateRef;
  serializeJson(mJsonDoc, &mLogBuffer, 200);
  debugMsg.data.data = (char *)mLogBuffer;
  debugMsg.data.capacity = strlen((const char *)mLogBuffer);
  debugMsg.data.size = sizeof(mLogBuffer);
  rcl_publish(mLogPublisher, &debugMsg, NULL);
}

void Odometry::RateControler(const float aRateRef, const float aRateEst,
                             int &aPwmRate, unsigned long &aPrevTime,
                             float &aPrevEpsilon, float &aIEpsilon) {
  float epsilon = abs(aRateRef) - abs(aRateEst);
  float d_epsilon = (epsilon - aPrevEpsilon) / (aPrevTime - millis());

  // reset and clamp integral (todo : add anti windup)
  if (aRateRef == 0.0) {
    aIEpsilon = 0.0;
  } else {
    aIEpsilon += epsilon * (aPrevTime - millis()) * RATE_CONTROLLER_KI;
  }
  aIEpsilon = constrain(aIEpsilon, -RATE_INTEGRAL_FREEZE, RATE_INTEGRAL_FREEZE);

  aPrevTime = millis();
  aPrevEpsilon = epsilon;

  aPwmRate = epsilon * RATE_CONTROLLER_KP + d_epsilon * RATE_CONTROLLER_KD +
             aIEpsilon * RATE_CONTROLLER_KI;

  // saturate output
  aPwmRate =
      constrain(aPwmRate, RATE_CONTROLLER_MIN_PWM, RATE_CONTROLLER_MAX_PWM);
}

float Odometry::RunningAverage(float mPrevAvg, const float aVal,
                               const int aSize) {
  return (mPrevAvg * (aSize - 1) + aVal) / aSize;
}
