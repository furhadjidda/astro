#ifndef  ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include <Encoder.h>

enum MotorInstance {
    MC_LEFT = 0,
    MC_RIGHT,
    MC_MAX         
};

class Encoder{
    public:
        Encoder
        (
        MotorInstance aInstance,
        mc::Encoder aEncoderObj
        ) 
        : mEncoderObj( aEncoderObj )
        {
        }

        virtual ~Encoder()
        {
        }
        
        int GetMotorDirectionVal()
        {
            if (mEncoderObj.getCountPerSecond() > 0) 
            {
                return 1;
            } 
            else 
            {
                return -1;
            }
        }
        
        int GetMotorIncValue()
        {
            return abs(mEncoderObj.getRawCount());
        }
        
        void SetMotorIncValue( int aVal )
        {
            mEncoderObj.resetCounter(aVal);
        }

    private:
        mc::Encoder mEncoderObj;
};  


#endif
