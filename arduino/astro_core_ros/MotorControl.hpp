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

#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H
#define USE_USBCON

#include <ros.h>
#include <Arduino.h>
#include <Encoder.h>
#include <DCMotor.h>
#include "Encoder.hpp"

class MotorControl : public Encoder
{
public:
  MotorControl
    (
    ros::NodeHandle &aNh,
    MotorInstance aInstance,
    mc::Encoder &aEncoderObj,
    mc::DCMotor &aMotor
    );

  virtual ~MotorControl()
  {
  }

  void InitMotorControl();

  void SetDutyCycle
    (
    int aVal
    );

private:
  mc::DCMotor mMotor;
  mc::Encoder mEncoder;
  MotorInstance mInstance;
  ros::NodeHandle& mNh;
};
#endif
