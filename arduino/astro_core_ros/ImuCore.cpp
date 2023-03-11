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

static const uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; //how often to read data from the board
static const uint16_t PRINT_DELAY_MS = 500; // how often to print the data
uint16_t printCount = 0; //counter to avoid printing every 10MS sample

//velocity = accel*dt (dt in seconds)
//position = 0.5*accel*dt^2
static const double ACCEL_VEL_TRANSITION =  (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
static const double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;
static const double DEG_2_RAD = 0.01745329251; //trig functions require radians, BNO055 outputs degrees


static char *dtostrf(double val, signed char width, unsigned char prec, char *sout)
{
    char fmt[20];
    sprintf(fmt, "%%%d.%df", width, prec);
    sprintf(sout, fmt, val);
    return sout;
}

void ImuCore::Init()
{
    if (!mBno055.begin())
    {
      mNodeHandle.logerror("Failed to initialize IMU!");
      while (1);
    }

    delay(1000);

    mBno055.setExtCrystalUse(true);

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
    mBno055.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    // Possible vector values can be:
    // - VECTOR_ACCELEROMETER - m/s^2
    // - VECTOR_MAGNETOMETER  - uT
    // - VECTOR_GYROSCOPE     - rad/s
    // - VECTOR_EULER         - degrees
    // - VECTOR_LINEARACCEL   - m/s^2
    // - VECTOR_GRAVITY       - m/s^2
    mXpos = mXpos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.x;
    mYpos = mYpos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.y;
    mHeadingVel = ACCEL_VEL_TRANSITION * linearAccelData.acceleration.x / cos(DEG_2_RAD * orientationData.orientation.x);

    mBno055.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    mBno055.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    mBno055.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    mBno055.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

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
    String calibrationMessage("Calibration:");
    calibrationMessage.concat(" Sys = " + String(system) +
                              " Gyro = " + String(gyro) +
                              " Accel = " + String(accel) +
                              " Mag = " + String(mag));
    mNodeHandle.loginfo(calibrationMessage.c_str());

    String positionMessage("Position :");
    positionMessage.concat(" Heading = " + String(orientationData.orientation.x) +
                           " Position = " + String(mXpos) + " , " + String(mYpos) + 
                           " Speed = " + String(mHeadingVel));
    mNodeHandle.loginfo(positionMessage.c_str());

    GetSensoreData();
}

void ImuCore::CalculateTemp()
{
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
}

void ImuCore::GetSensoreData()
{
    sensor_t sensor;
    mBno055.getSensor(&sensor);
    String sensorInfo("Sensor Information :");
    sensorInfo.concat(" Sensor = " + String(sensor.name) +
                      " Driver Version = " + String(sensor.version) + " Unique ID = " + String(sensor.sensor_id));
    mNodeHandle.loginfo(sensorInfo.c_str());
}