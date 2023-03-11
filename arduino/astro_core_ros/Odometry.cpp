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
#include <ArduinoHardware.h>
#include "Odometry.hpp"

    void Odometry::InitNode()
    {
        mRightMotor = new MotorControl( mNodeHandle, MC_RIGHT, encoder1, M1);
        mLeftMotor = new MotorControl( mNodeHandle, MC_LEFT, encoder2, M2);
        // Initialize Motors
        mRightMotor->InitMotorControl();
        mLeftMotor->InitMotorControl();
    }

    void Odometry::CommandVelocityCallback(const geometry_msgs::Twist& msg) 
    {
        double linear = msg.linear.x;
        double angular = msg.angular.z;
        mLeftSpeed = linear - angular * mWheelDistance / 2.0;
        mRightSpeed = linear + angular * mWheelDistance / 2.0;
        String message = "CommandVelocityCallback linear = " + String(linear);
        mNodeHandle.loginfo(message.c_str());
        double left_duty_cycle = ( (linear - 0.5 * angular) / 1) * 100 ;
        double right_duty_cycle = ( (linear + 0.5 * angular) / 1) * 100;
        mLeftMotor->SetDutyCycle(left_duty_cycle);
        mRightMotor->SetDutyCycle(right_duty_cycle);
    }

    void Odometry::UpdateOdometry()
    {
        ros::Time current_time = mNodeHandle.now();;
        double elapsed_time = (current_time - mLastTime).toSec();
        int left_encoder_count_ = mLeftMotor->GetMotorIncValue();
        int right_encoder_count_ = mRightMotor->GetMotorIncValue();
        double left_distance = ((left_encoder_count_ - mLastLeftEncoderCount) / static_cast<double>(mEncoderResolution)) * 2 * M_PI * mWheelRadius;
        double right_distance = ((right_encoder_count_ - mLastRightEncoderCount) / static_cast<double>(mEncoderResolution)) * 2 * M_PI * mWheelRadius;

        mLastLeftEncoderCount = left_encoder_count_;
        mLastRightEncoderCount = right_encoder_count_;

        double linear_distance = (left_distance + right_distance) / 2.0;
        double angular_distance = (right_distance - left_distance) / mWheelDistance;

        double linear_velocity = linear_distance / elapsed_time;
        double angular_velocity = angular_distance / elapsed_time;

        mX += linear_distance * cos(mTheta + angular_distance / 2.0);
        mY += linear_distance * sin(mTheta + angular_distance / 2.0);
        mTheta += angular_distance;

        mLastTime = current_time;

        double left_speed_error = mLeftSpeed - linear_velocity;
        double right_speed_error = mRightSpeed - linear_velocity;

        double left_speed_correction = 0;// pid_left_speed_.compute(left_speed_error, elapsed_time);
        double right_speed_correction = 0; //pid_right_speed_.compute(right_speed_error, elapsed_time);

        // Set the motor speeds based on the PID correction
        // This assumes that you have some way of setting the motor speed
        // based on the encoder feedback (e.g. a motor controller)
        //setLeftMotorSpeed(linear_velocity + left_speed_correction);
        //setRightMotorSpeed(linear_velocity + right_speed_correction);
        String message = " >>> UpdateOdometry :: linear_velocity  = " + String(linear_velocity) + "left encoder = " + String(left_encoder_count_) + " right encoder = " + String(right_encoder_count_);
        mNodeHandle.loginfo(message.c_str());
        //mLeftMotor->SetDutyCycle(linear_velocity + left_speed_correction);
        //mRightMotor->SetDutyCycle(linear_velocity + right_speed_correction);
    }

    void Odometry::GetOdomData(  nav_msgs::Odometry& aOdom )
    {
        static tf::TransformBroadcaster broadcaster;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(mTheta);

        // Publish the odometry message
        nav_msgs::Odometry odom;
        odom.header.stamp = mNodeHandle.now();;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";
        odom.pose.pose.position.x = mX;
        odom.pose.pose.position.y = mY;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        odom.twist.twist.linear.x = (mLeftSpeed + mRightSpeed) / 2.0;
        odom.twist.twist.angular.z = (mRightSpeed - mLeftSpeed) / mWheelDistance;
        //odom_pub_.publish(odom);
        aOdom = odom;

        // Publish the transform from "odom" to "base_footprint"
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = mNodeHandle.now();;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_footprint";
        odom_trans.transform.translation.x = mX;
        odom_trans.transform.translation.y = mY;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;
        //broadcaster.sendTransform(odom_trans);
    }