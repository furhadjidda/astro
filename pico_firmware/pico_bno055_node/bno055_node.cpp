#include "bno055.hpp"
#include "hardware/i2c.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "pico_uart_transports.h"
#include "rcl/time.h"
#include "rcutils/time.h"
#include <cstdio>
#include <cstring>
#include <math.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/string.h>
// Structure for quaternion
struct Quaternion {
    float x;
    float y;
    float z;
    float w;
};

float previous_yaw = 0.0;
float previous_pitch = 0.0;
float previous_roll = 0.0;
// Micro-ROS Configuration
#define LED_PIN 25
#define PUBLISH_RATE_HZ 5 // Publishing frequency

bno055_sensor::Bno055 imu;

sensor_msgs__msg__Imu imu_msg;

rcl_publisher_t imu_publisher;
rcl_publisher_t imu_debug_publisher;
std_msgs__msg__String imu_data_msg;

char json_buffer[1024];

bool init_bno055() {
    if (!imu.initialization()) {
        printf("BNO055 initialization failed!\n");
        return false;
    }
    imu.set_ext_crystal_use(true);
    printf("BNO055 initialized successfully.\n");
    return true;
}

// Function to convert Euler angles to quaternion
Quaternion eulerToQuaternion(float roll, float pitch, float yaw) {
    Quaternion q;

    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

void populate_imu_msg(sensor_msgs__msg__Imu &msg, std_msgs__msg__String &debug_msg) {
    double accel[3] = {0.0, 0.0, 0.0};
    double gyro[3] = {0.0, 0.0, 0.0};
    double mag[3] = {0.0, 0.0, 0.0};
    double euler[3] = {0.0, 0.0, 0.0};
    quaternion_data q = {};
    uint8_t calibration[4] = {0};
    // get_calibration(uint8_t *sys, uint8_t *gyro, uint8_t *accel, uint8_t *mag)
    imu.get_calibration(&calibration[0], &calibration[1], &calibration[3], &calibration[4]);

    imu.get_vector(VECTOR_ACCELEROMETER, accel);
    imu.get_vector(VECTOR_GYROSCOPE, gyro);
    imu.get_vector(VECTOR_MAGNETOMETER, mag);
    imu.get_vector(VECTOR_EULER, euler);
    imu.get_quaternion(q);

    msg.header.frame_id.data = "imu_frame";
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

    // Fill orientation (optional: needs quaternion calculation)
    // Quaternion q = eulerToQuaternion(euler[0], euler[1], euler[2]);
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
    std::snprintf(json_buffer, sizeof(json_buffer),
                  "{\"accelerometer\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f},"
                  "\"gyroscope\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f},"
                  "\"magnetometer\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f},"
                  "\"euler\":{\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f},"
                  "\"quaternion\":{\"w\":%.2f,\"x\":%.2f,\"y\":%.2f,\"z\":%.2f},"
                  "\"calibration\":{\"sys\":%d,\"gyro\":%d,\"accel\":%d,\"mag\":%d}}",
                  accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2], mag[0], mag[1], mag[2], euler[0], euler[1],
                  euler[2], q.w, q.x, q.y, q.z, calibration[0], calibration[1], calibration[2], calibration[3]);

    debug_msg.data.data = strdup(json_buffer);
    debug_msg.data.size = strlen(json_buffer);
    debug_msg.data.capacity = debug_msg.data.size + 1;
}
int main() {
    setenv("ROS_DOMAIN_ID", "10", 1);
    rmw_uros_set_custom_transport(true, NULL, pico_serial_transport_open, pico_serial_transport_close,
                                  pico_serial_transport_write, pico_serial_transport_read);

    // Initialize BNO055
    if (!init_bno055()) {
        return 1;
    }
    cyw43_arch_init();
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    // Initialize micro-ROS
    rcl_allocator_t allocator = rcl_get_default_allocator();
    // Initialize and modify options (Set DOMAIN ID to 10)
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, 10);

    // Initialize rclc support object with custom option
    rclc_support_t support;
    rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

    rcl_node_t node;
    rclc_node_init_default(&node, "_fram", "", &support);

    rclc_publisher_init_default(&imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
                                "imu/data_raw");
    rclc_publisher_init_default(&imu_debug_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                                "imu/debug");
    // imu_debug_publisher

    rcl_timer_t timer;
    rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(1000 / PUBLISH_RATE_HZ),
                            [](rcl_timer_t *timer, int64_t last_call_time) {
                                populate_imu_msg(imu_msg, imu_data_msg);
                                rcl_publish(&imu_publisher, &imu_msg, NULL);
                                rcl_publish(&imu_debug_publisher, &imu_data_msg, NULL);
                                cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, !cyw43_arch_gpio_get(CYW43_WL_GPIO_LED_PIN));
                            });

    rclc_executor_t executor;
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);

    printf("Starting micro-ROS publisher...\n");
    while (rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)) == RCL_RET_OK) {
        // Main loop spins the executor
    }

    // Clean up
    rcl_publisher_fini(&imu_publisher, &node);
    rcl_node_fini(&node);
    rclc_support_fini(&support);

    return 0;
}
