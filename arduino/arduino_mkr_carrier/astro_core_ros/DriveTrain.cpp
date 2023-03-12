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
#include "DriveTrain.hpp"


template <typename type>
type sign(type value) 
{
    return type((value>0)-(value<0));
}

DriveTrain::DriveTrain
    (
    ros::NodeHandle& nodeHandle
    )
    : mNodeHandle( nodeHandle )
{
}

void DriveTrain::InitNode()
{
    InitMotorCarrier();
    mRightMotor = new MotorControl( mNodeHandle, MC_RIGHT, encoder1, M1);
    mLeftMotor = new MotorControl( mNodeHandle, MC_LEFT, encoder2, M2);
    mRightMotor->InitMotorControl();
    mLeftMotor->InitMotorControl();    
    mFreqRate.start((unsigned long) millis());
    mFreqOdometry.start((unsigned long) millis());
    mFreqController.start((unsigned long) millis());
}


void DriveTrain::CommandVelocityCallback(const geometry_msgs::Twist& aMsg) 
{
    mLinearVelocityRef  = aMsg.linear.x;
    mAngularVelocityRef = aMsg.angular.z;
    // Vr = (2V + Wb * W) / 2R
    // Vl = (2V - Wb * W) / 2R
    // Vr is the right wheel speed
    // Vl is the left wheel speed
    // V is the linear velocity of the robot
    // W is the angular velocity (in radians per second) of the robot
    // R is the radius of the turning circle (equal to V / W)
    // Wb is the wheelbase length

    // MIXER
    mMotorRight_RateRef 
        = (mLinearVelocityRef + BASE_LENGTH / 2.f * mAngularVelocityRef) / (WHEEL_RADIUS);

    mMotorLeft_RateRef 
        = (mLinearVelocityRef - BASE_LENGTH / 2.f * mAngularVelocityRef) / (WHEEL_RADIUS);
}


void DriveTrain::UpdateOdometry()
{
    // Refer : http://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf
    if( mFreqRate.delay( millis() ) ) 
    {
        float dt;
    
        // MOTOR RIGHT
        // direction
        // Pass it through median filter to remove noise  
        mMotorRight_DirMedianFilter.in( mRightMotor->GetMotorDirectionVal() );
        mMotorRight_FilteredDirection = mMotorRight_DirMedianFilter.out();
    
        // filter increment per second
        dt = ( millis() - mMotorRight_PrevTime );
        mMotorRight_PrevTime = millis();
        // Calculate Motor Encoder Increment Value / per second
        mMotorRight_FilteredIncrementPerSecond 
            = RunningAverage
                (
                mMotorRight_FilteredIncrementPerSecond,
                (float)mRightMotor->GetMotorIncValue() / dt * 1000.0f,
                RATE_AVERAGE_FILTER_SIZE
                );
    
        // Estimating Rate
        mMotorRight_RateEst 
            = (float)mMotorRight_FilteredDirection * mMotorRight_FilteredIncrementPerSecond * RATE_CONV;

        // Clear encoder increments
        mRightMotor->SetMotorIncValue( 0 );
        
        String messageR = "mMotorRight_RateEst Right = " + String(mMotorRight_RateEst);
        mNodeHandle.logdebug(messageR.c_str());
        
    
        if ( abs(mMotorRight_RateEst) < 0.1f )
        {
            mMotorRight_RateEst = 0.0f;
        }
    
        mRightMotor->SetMotorDirectionVal( 0 );
        // MOTOR LEFT
        // direction
        mMotorLeft_DirMedianFilter.in( mLeftMotor->GetMotorDirectionVal() );
        mMotorLeft_FilteredDirection = mMotorLeft_DirMedianFilter.out();
    
        // filter increment per second
        dt = ( millis() - mMotorLeft_PrevTime );
        mMotorLeft_PrevTime = millis();
        
        mMotorLeft_FilteredIncrementPerSecond 
            = RunningAverage
                (
                mMotorLeft_FilteredIncrementPerSecond,
                (float)mLeftMotor->GetMotorIncValue() / dt * 1000.0f,
                RATE_AVERAGE_FILTER_SIZE
                );
    
        // estimated rate
        mMotorLeft_RateEst
            = (float)mMotorLeft_FilteredDirection * mMotorLeft_FilteredIncrementPerSecond * RATE_CONV;
        mLeftMotor->SetMotorIncValue( 0 );
        
        String messageL = "mMotorLeft_RateEst Right = " + String(mMotorLeft_RateEst);
        mNodeHandle.logdebug(messageL.c_str());
    
        if (abs(mMotorLeft_RateEst) < 0.1f)
        {
            mMotorLeft_RateEst = 0.0f;
        }
        mLeftMotor->SetMotorDirectionVal( 0 );
    }

    if( mFreqController.delay( millis() ) ) 
    {
        // MOTOR RIGHT
        RateControler
            (
            mMotorRight_RateRef,
            mMotorRight_RateEst,
            mMotorRight_PwmRate,
            mController_MotorRightPrevTime,
            mController_MotorRightPrevEpsilon,
            mController_MotorRightInt
            );

        mMotorRightPwm = mMotorRightPwm + mMotorRight_PwmRate;
        mMotorRightPwm = constrain(mMotorRightPwm, 0, PWM_MAX);
        String messageR = "Pwm Right = " + String(mMotorRightPwm);
        mNodeHandle.logdebug(messageR.c_str());

        // MOTOR LEFT
        RateControler
            (
            mMotorLeft_RateRef,
            mMotorLeft_RateEst,
            mMotorLeft_PwmRate,
            mController_MotorLeftPrevTime,
            mController_MotorLeftPrevEpsilon,
            mController_MotorLeftInt
            );
        mMotorLeftPwm = mMotorLeftPwm + mMotorLeft_PwmRate;
        mMotorLeftPwm = constrain(mMotorLeftPwm, 0, PWM_MAX);
        String messageL = "Pwm Left = " + String(mMotorRightPwm);
        mNodeHandle.logdebug(messageL.c_str());
               
        double leftDutyCycle = (mLinearVelocityRef - 0.5 * mAngularVelocityRef) * 100 ;
        double rightDutyCycle = (mLinearVelocityRef + 0.5 * mAngularVelocityRef)* 100;
        mLeftMotor->SetDutyCycle(leftDutyCycle);
        mRightMotor->SetDutyCycle(rightDutyCycle);
    }

    // Printing System Data for Motor Carrier
    float batteryVoltage = (float)battery.getConverted();
    String message = "Battery voltage: " + String(batteryVoltage) + "V , Firmware Verison : " + controller.getFWVersion();
    mNodeHandle.loginfo(message.c_str());
}

void DriveTrain::GetOdomData(nav_msgs::Odometry &aOdom)
{
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
    mOdom.header.stamp = mNodeHandle.now();
    mOdom.header.frame_id = "odom";
    mOdom.child_frame_id = "base_link";
    mOdom.pose.pose.position.x += dx;
    mOdom.pose.pose.position.y += dy;
    mOdom.pose.pose.position.z = 0.0;
    mOdom.pose.pose.orientation.w = qw;
    mOdom.pose.pose.orientation.x = qx;
    mOdom.pose.pose.orientation.y = qy;
    mOdom.pose.pose.orientation.z = qz;
    // Velocity expressed in base_link frame
    mOdom.twist.twist.linear.x = mLinearVelocityEst;
    mOdom.twist.twist.linear.y = 0.0f;
    mOdom.twist.twist.angular.z = mAngularVelocityEst;

    aOdom = mOdom;

}

void DriveTrain::RateControler
    (
    const float aRateRef,
    const float aRateEst,
    int & aPwmRate,
    unsigned long & aPrevTime,
    float & aPrevEpsilon,
    float & aIEpsilon
    )
{
    double deltaTime = (double)(millis() - aPrevTime);
    float epsilon = abs(aRateRef) - abs(aRateEst);
    float d_epsilon = (epsilon - aPrevEpsilon) / deltaTime;

    // reset and clamp integral (todo : add anti windup)
    if (aRateRef == 0.0) 
    {
        aIEpsilon = 0.0;
    } 
    else 
    {
        aIEpsilon += epsilon * deltaTime;
    }
    aIEpsilon = constrain(aIEpsilon, -RATE_INTEGRAL_FREEZE, RATE_INTEGRAL_FREEZE);

    aPrevTime = millis();
    aPrevEpsilon = epsilon;

    aPwmRate = ( epsilon * RATE_CONTROLLER_KP ) + ( d_epsilon * RATE_CONTROLLER_KD ) + ( aIEpsilon * RATE_CONTROLLER_KI );
    
    String messepsilon = "epsilon = " + String(epsilon) + " d_epsilon = " + String(d_epsilon) + " aIEpsilon = " + String(aIEpsilon) + " deltaTime = " + String(deltaTime);
    mNodeHandle.logdebug(messepsilon.c_str());

    // saturate output
    aPwmRate = constrain(aPwmRate, RATE_CONTROLLER_MIN_PWM, RATE_CONTROLLER_MAX_PWM);
}


float DriveTrain::RunningAverage
    (
    float mPrevAvg,
    const float aVal,
    const int aSize
    )
{
    return (mPrevAvg * (aSize - 1) + aVal) / aSize;
}

void DriveTrain::InitMotorCarrier()
{
    if (controller.begin())
    {
        String message = "Motor Carrier connected, firmware version : " + controller.getFWVersion();
    }
    else
    {
        mNodeHandle.logerror("Couldn't connect! Is the red LED blinking? You may need to update the firmware with FWUpdater sketch");
        while (1)
            ;
    }

    // Reboot the motor controller; brings every value back to default
    mNodeHandle.loginfo("reboot");
    controller.reboot();
    delay(500);

    // Reset the encoder internal counter to zero (can be set to any initial value)
    mNodeHandle.loginfo("reset counters");
    encoder1.resetCounter(0);
    encoder2.resetCounter(0);

    // Take the battery status
    float batteryVoltage = (float)battery.getConverted();
    String message = "Battery voltage: " + String(batteryVoltage) + "V";
    mNodeHandle.loginfo(message.c_str());

    delay(1000);
}