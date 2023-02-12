#include <stdint.h>

#include "MotorControl.hpp"

MotorControl::MotorControl
    (
    ros::NodeHandle& aNh,
    MotorInstance aInstance,
    mc::Encoder& aEncoderObj,
    mc::DCMotor& aMotor
    ): Encoder( aInstance, aEncoderObj )
    , mMotor( aMotor )
    , mEncoder( aEncoderObj )
    , mInstance( aInstance )
    , mNh( aNh )
{
}


void MotorControl::InitMotorControl()
{
  String motormessage =  "InitMotorControl for motor " + String(mInstance);
  mNh.logwarn( motormessage.c_str() );
  mMotor.setDuty(0);
}

void MotorControl::SetMotorRateAndDirection
    (
    int aPwmRef,// Controls the speed
    const float aRateRef // Controls the direction
    )
{
      // avoid noisy pwm range
      if (abs(aRateRef) < 0.1)
        aPwmRef = 0;
    
      int duty = 50;
      duty = duty * aRateRef;
      if(duty > 100)
      {
        duty = 100;
      }
      if( duty < -100)
      {
        duty = -100;
      }
      String message =  "Setting aRateRef = " + String(aRateRef) + " aPwmRef = " + String(aPwmRef) + " Duty =" + String(duty) + " for motor " + String(mMotor.getInstanceId()) + " encoder = " + String(mEncoder.getInstance());
      mNh.loginfo( message.c_str() );
      mMotor.setDuty( duty );
}
