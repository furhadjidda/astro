#include "bno055.hpp"
#include "hardware/i2c.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "pico_uart_transports.h"
#include <math.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/imu.h>

// Structure for quaternion
struct Quaternion {
    float x;
    float y;
    float z;
    float w;
};

// Micro-ROS Configuration
#define LED_PIN 25
#define PUBLISH_RATE_HZ 5 // Publishing frequency

bno055_sensor::Bno055 imu;

sensor_msgs__msg__Imu imu_msg;

rcl_publisher_t imu_publisher;

bool init_bno055() {
    if (!imu.initialization()) {
        printf("BNO055 initialization failed!\n");
        return false;
    }
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

void populate_imu_msg(sensor_msgs__msg__Imu &msg) {
    double accel[3] = {0.0, 0.0, 0.0};
    double gyro[3] = {0.0, 0.0, 0.0};
    double mag[3] = {0.0, 0.0, 0.0};
    double euler[3] = {0.0, 0.0, 0.0};

    imu.get_vector(VECTOR_ACCELEROMETER, accel);
    imu.get_vector(VECTOR_GYROSCOPE, gyro);
    imu.get_vector(VECTOR_MAGNETOMETER, mag);
    imu.get_vector(VECTOR_EULER, euler);

    msg.header.frame_id.data = "imu_frame";
    rcl_time_point_value_t now;
    msg.header.stamp.sec = RCL_NS_TO_S(now);
    msg.header.stamp.nanosec = now % RCL_S_TO_NS(1);

    // Fill accelerometer data
    msg.linear_acceleration.x = accel[0];
    msg.linear_acceleration.y = accel[1];
    msg.linear_acceleration.z = accel[2];

    // Fill gyroscope data
    msg.angular_velocity.x = gyro[0];
    msg.angular_velocity.y = gyro[1];
    msg.angular_velocity.z = gyro[2];

    // Fill orientation (optional: needs quaternion calculation)
    Quaternion q = eulerToQuaternion(euler[0], euler[1], euler[2]);
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
}
int main() {

    rmw_uros_set_custom_transport(true, NULL, pico_serial_transport_open, pico_serial_transport_close,
                                  pico_serial_transport_write, pico_serial_transport_read);

    // Initialize BNO055
    if (!init_bno055()) {
        return 1;
    }
    cyw43_arch_init();
    // Initialize micro-ROS
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    rclc_support_init(&support, 0, NULL, &allocator);

    rcl_node_t node;
    rclc_node_init_default(&node, "bno055_publisher", "", &support);

    rclc_publisher_init_default(&imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data");

    rcl_timer_t timer;
    rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(1000 / PUBLISH_RATE_HZ),
                            [](rcl_timer_t *timer, int64_t last_call_time) {
                                populate_imu_msg(imu_msg);
                                rcl_publish(&imu_publisher, &imu_msg, NULL);
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
