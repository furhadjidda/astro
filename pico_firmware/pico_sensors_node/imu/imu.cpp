/*
 *   This file is part of astro.
 *
 *   astro is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   astro is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License v3.0
 *   along with astro.  If not, see <https://www.gnu.org/licenses/>.
 */
#include "imu.hpp"

#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/string_functions.h>

bool IMUSensor::initialize(i2c_inst_t *aI2cInstance) {
    if (!imu->initialization()) {
        printf("BNO055 initialization failed!\n");
        return false;
    }
    printf("BNO055 initialized successfully.\n");
    CalibrationData data = {};
    _flashManager.read_data(data, sizeof(data));
    // Wait before setting calibration data
    sleep_ms(3000);
    imu->set_calibration_data(data);
    return true;
}

void IMUSensor::get_debug_message(std_msgs__msg__String &msg) {
    rosidl_runtime_c__String__assign(&msg.data, debug_msg_buffer);
}

void IMUSensor::get_imu_data(sensor_msgs__msg__Imu &msg) {
    double accel[3] = {0.0, 0.0, 0.0};
    double gyro[3] = {0.0, 0.0, 0.0};
    double mag[3] = {0.0, 0.0, 0.0};
    double euler[3] = {0.0, 0.0, 0.0};
    quaternion_data q = {};
    uint8_t calibration[4] = {0};
    // get_calibration(uint8_t *sys, uint8_t *gyro, uint8_t *accel, uint8_t *mag)
    imu->get_calibration(&calibration[0], &calibration[1], &calibration[2], &calibration[3]);
    imu->get_vector(VECTOR_ACCELEROMETER, accel);
    imu->get_vector(VECTOR_GYROSCOPE, gyro);
    imu->get_vector(VECTOR_MAGNETOMETER, mag);
    imu->get_vector(VECTOR_EULER, euler);
    imu->get_quaternion(q);

    rosidl_runtime_c__String__assign(&msg.header.frame_id, "imu_frame");
    rcl_time_point_value_t now;
    rcutils_time_point_value_t time_now;
    rcutils_system_time_now(&time_now);
    now = static_cast<rcl_time_point_value_t>(time_now);
    // Populate the timestamp
    msg.header.stamp.sec = static_cast<uint32_t>(now / RCL_S_TO_NS(1));
    msg.header.stamp.nanosec = static_cast<uint32_t>(now % RCL_S_TO_NS(1));

    // Fill accelerometer data
    msg.linear_acceleration.x = accel[0];
    msg.linear_acceleration.y = accel[1];
    msg.linear_acceleration.z = accel[2];

    // Fill gyroscope data
    msg.angular_velocity.x = gyro[0];
    msg.angular_velocity.y = gyro[1];
    msg.angular_velocity.z = gyro[2];

    // Fill orientation
    msg.orientation.x = q.x;
    msg.orientation.y = q.y;
    msg.orientation.z = q.z;
    msg.orientation.w = q.w;

    // Add covariance if needed
    // For simplicity, leaving covariances as zero
    for (int i = 0; i < 9; ++i) {
        msg.linear_acceleration_covariance[i] = 0.0;
        msg.angular_velocity_covariance[i] = 0.0;
        msg.orientation_covariance[i] = 0.0;
    }

    uint8_t system = 0;
    uint8_t seltest = 0;
    uint8_t error = 0;
    imu->get_system_status(&system, &seltest, &error);

    std::snprintf(debug_msg_buffer, sizeof(debug_msg_buffer),
                  "{\"system_stats\":{\"sys\":%d,\"self_test\":%d,\"error\":%d},"
                  "\"calibration\":{\"sys\":%d,\"gyro\":%d,\"accel\":%d,\"mag\":%d}}",
                  system, seltest, error, calibration[0], calibration[1], calibration[2], calibration[3]);
}