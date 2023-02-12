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

#include "ImuCore.hpp"

static char *dtostrf(double val, signed char width, unsigned char prec, char *sout)
{
    char fmt[20];
    sprintf(fmt, "%%%d.%df", width, prec);
    sprintf(sout, fmt, val);
    return sout;
}

void ImuCore::Init()
{
    // Node Handle Instance for logging
    // mNodeHandle = aNh;

    if (!mBno055.begin())
    {
      mNodeHandle.logerror("Failed to initialize IMU!");
      while (1);
    }

    delay(1000);

    /* Display the current temperature */
    int8_t temp = mBno055.getTemp();
    char buf[64] = {0};
    char val[16] = {0};
    strcpy_P(buf, (const char *)F("Current Temperature = "));
    // Adding  temperature to val buffer
    dtostrf(temp, 8, 4, val);
    // Concatenating val to buf
    strcat(buf, val);
    // Concatenating " C" to buf
    strcat(buf, " C");
    mNodeHandle.loginfo(buf);

    mBno055.setExtCrystalUse(true);

    mNodeHandle.loginfo("Calibration status values: 0=Uncalibrated, 3=Fully Calibrated");
}

void ImuCore::GetIMUData(sensor_msgs::Imu &aImuData)
{
    sensors_event_t orientationData;
    sensors_event_t angVelocityData;
    sensors_event_t linearAccelData;
    sensors_event_t magnetometerData;
    sensors_event_t accelerometerData;
    sensors_event_t gravityData;
    mBno055.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    mBno055.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    mBno055.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    mBno055.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    mBno055.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    mBno055.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
    // Possible vector values can be:
    // - VECTOR_ACCELEROMETER - m/s^2
    // - VECTOR_MAGNETOMETER  - uT
    // - VECTOR_GYROSCOPE     - rad/s
    // - VECTOR_EULER         - degrees
    // - VECTOR_LINEARACCEL   - m/s^2
    // - VECTOR_GRAVITY       - m/s^2
    imu::Quaternion quat = mBno055.getQuat();
    aImuData.orientation.x = quat.x();
    aImuData.orientation.y = quat.y();
    aImuData.orientation.z = quat.z();
    aImuData.orientation.w = quat.w();

    aImuData.linear_acceleration.x = linearAccelData.acceleration.x;
    aImuData.linear_acceleration.y = linearAccelData.acceleration.y;
    aImuData.linear_acceleration.z = linearAccelData.acceleration.z;

    aImuData.angular_velocity.x = angVelocityData.gyro.x;
    aImuData.angular_velocity.y = angVelocityData.gyro.y;
    aImuData.angular_velocity.z = angVelocityData.gyro.z;

    uint8_t system, gyro, accel, mag = 0;
    mBno055.getCalibration(&system, &gyro, &accel, &mag);
    String calibrationMessage("CALIBRATION:");
    calibrationMessage.concat(" Sys = " + String(system) +
                              " Gyro = " + String(gyro) +
                              " Accel = " + String(accel) +
                              " Mag = " + String(mag));
    mNodeHandle.loginfo(calibrationMessage.c_str());
}