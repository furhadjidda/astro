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

#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
enum MotorInstance {
    MC_LEFT = 0,
    MC_RIGHT,
    MC_MAX         
};

class Encoder{
    public:
        Encoder
            (
            byte aEncoderPinA,
            byte aEncoderPinB,            
            MotorInstance aInstance
            );

        virtual ~Encoder()
        {
        }

        void SetMotorCheckDirectionVal( int aVal )
        {
            mMotorCheckDirection = aVal;
        }
        
        void SetMotorDirectionVal( int aVal )
        {
            mMotorDirection = aVal;
        }

        int GetMotorCheckDirectionVal()
        {
            return mMotorCheckDirection;
        }
        
        int GetMotorDirectionVal()
        {
            return mMotorDirection;
        }
        
        int GetMotorIncValue()
        {
            return mMotorInc;
        }
        
        void SetMotorIncValue( int aVal )
        {
            mMotorInc = aVal;
        }
        
        void IsrLeftCounterDirection();

        void IsrRightCounterDirection();

    private:

        volatile int mMotorInc = 0;
        int mMotorDirection;
        int mMotorCheckDirection;
        byte mEncoderPinA;
        byte mEncoderPinB;
};  


#endif
