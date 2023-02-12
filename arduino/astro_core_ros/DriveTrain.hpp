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
        ros::NodeHandle mNodeHandle;    
        //ros::Subscriber<geometry_msgs::Twist,DriveTrain> mTeleopSubscriber;    
        
        /* DEBUG */
        geometry_msgs::Point mDebugLeft;
        ros::Publisher mDebugLeftPub{"debug_left", &mDebugLeft};
        geometry_msgs::Point mDebugRight;
        ros::Publisher mDebugRightPub{"debug_right", &mDebugRight};
    
        /* Odometry publisher */
        nav_msgs::Odometry mOdom;
        ros::Publisher mOdomPublisher{"odom", &mOdom};

        sensor_msgs::Imu mImuData;
        ros::Publisher mImuPub{"imu",&mImuData};

        //ImuProcessor mProcessor;
};


#endif
