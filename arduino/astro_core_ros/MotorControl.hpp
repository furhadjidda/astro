#ifndef  MOTORCONTROL_H
#define MOTORCONTROL_H
#define USE_USBCON

#include <ros.h>
#include <Arduino.h>
#include <Encoder.h>
#include <DCMotor.h>
#include "Encoder.hpp"

class MotorControl: public Encoder{
    public:
        MotorControl
          (
          ros::NodeHandle& aNh,
          MotorInstance aInstance,
          mc::Encoder& aEncoderObj,
          mc::DCMotor& aMotor
          );
            
        virtual ~MotorControl()
        {            
        }

        void InitMotorControl();
        
        void SetMotorRateAndDirection
            (
            int aPwmRef,
            const float aRateRef
            );

    private:
        mc::DCMotor mMotor;
        mc::Encoder mEncoder;
        MotorInstance mInstance;
        ros::NodeHandle mNh;
};
#endif
