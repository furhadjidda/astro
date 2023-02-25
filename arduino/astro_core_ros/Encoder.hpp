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

#ifndef  ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include <Encoder.h>

enum MotorInstance 
{
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

        void SetMotorDirectionVal( int aVal )
        {
            mMotorDirection = aVal;
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
        int mMotorDirection;
        int mMotorCheckDirection;
};  


#endif
