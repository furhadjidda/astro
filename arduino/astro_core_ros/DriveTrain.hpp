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

#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <Timer.h>
#include <MedianFilter.h>
#include <ArduinoMotorCarrier.h>
#include "MotorControl.hpp"
#include "RoverControllerConfig.h"


class DriveTrain{
    public:
        DriveTrain( ros::NodeHandle& nodeHandle );
    
        virtual ~DriveTrain()
        {
        }

        void UpdateOdometry();

        void UpdateOdometry2();
        
        void InitNode();

       void GetOdomData(  nav_msgs::Odometry& aOdom )
       {
          PublishOdom();
          aOdom = mOdom;
       }

      void CommandVelocityCallback( const geometry_msgs::Twist& aMsg );

    private:
        void RateControler
            (
            const float aRateRef,
            const float aRateEst,
            int & aPwmRate,
            unsigned long & aPrevTime,
            float & aPrevEpsilon,
            float & aIEpsilon
            );
        
        float RunningAverage
            (
            float mPrevAvg,
            const float aVal,
            const int aSize
            );

        void PublishOdom();
        void PublishOdom2();
        void PublishImu();

        /* Define frequency loops */
        Timer mFreqRate{FREQUENCY_RATE};
        Timer mFreqOdometry{FREQUENCY_ODOMETRY};
        Timer mFreqRosSpinOnce{FREQUENCY_ROSPINONCE};
        Timer mFreqController{FREQUENCY_CONTROLLER};



        /* Define median filter for direction */
        // https://en.wikipedia.org/wiki/Median_filter#Worked_one-dimensional_example
        // The median filter is a non-linear digital filtering technique, often used 
        // to remove noise from an image or signal.
        MedianFilter mMotorRight_DirMedianFilter{RATE_DIRECTION_MEDIAN_FILTER_SIZE};
        MedianFilter mMotorLeft_DirMedianFilter{RATE_DIRECTION_MEDIAN_FILTER_SIZE};

            
        /* Mixer variable */
        float mLinearVelocityRef;
        float mAngularVelocityRef;


        /* Mixer variable */
        float mLinearVelocityEst;
        float mAngularVelocityEst;

        // right
        MotorControl* mRightMotor;
        int mMotorRight_FilteredDirection;
        float mMotorRight_FilteredIncrementPerSecond;
        float mMotorRight_RateEst;
        float mMotorRight_RateRef;
        int mMotorRight_PwmRate;
        unsigned long mMotorRight_PrevTime;
        int mMotorRightPwm = 0;
    
        // left
        MotorControl* mLeftMotor;
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

        // ROS Variables
        ros::NodeHandle& mNodeHandle;    
    
        // Odometry publisher
        nav_msgs::Odometry mOdom;
        ros::Publisher mOdomPublisher{"odom", &mOdom};
};


#endif
