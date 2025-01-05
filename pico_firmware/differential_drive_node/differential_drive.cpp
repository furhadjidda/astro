/*
 *   This file is part of the astro project.
 *
 *   Astro project is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   Astro project is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License v3.0
 *   along with astro project.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "hardware/i2c.h"
#include "motor_control.hpp"
#include "odometry.hpp"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "pico_uart_transports.h"
#include <geometry_msgs/msg/twist.h>
#include <memory>
#include <nav_msgs/msg/odometry.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/string.h>
#include <string>

// ROS Subscriber
rcl_subscription_t cmd_vel_subscriber;
Odometry odometry;
motor_control motor;

// ROS Publishers
rcl_publisher_t odometry_publisher;
rcl_publisher_t debug_publisher;

// Odometry message
nav_msgs__msg__Odometry odometry_msg;
geometry_msgs__msg__Twist msg;

// Debug message
std_msgs__msg__String debug_msg;

// Callback function for /cmd_vel messages
void cmd_vel_callback(const void *msg_in) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msg_in;

    // Extract linear and angular velocity
    float linear_x = msg->linear.x;
    float angular_z = msg->angular.z;

    // Debug print
    printf("Received /cmd_vel - Linear: %.2f, Angular: %.2f\n", linear_x, angular_z);

    // Publish debug message
    std::string debug_info =
        "Processed /cmd_vel: Linear=" + std::to_string(linear_x) + ", Angular=" + std::to_string(angular_z);
    if (!rosidl_runtime_c__String__assign(&debug_msg.data, debug_info.c_str())) {
        printf("Failed to assign debug message\n");
        return;
    }

    if (RCL_RET_OK != rcl_publish(&debug_publisher, &debug_msg, NULL)) {
        printf("Failed to publish debug message\n");
    }

    // Take action based on received velocities
    // For example, control motor speeds or other actuators here
    // ...
    odometry.cmd_vel_callback(msg_in);
}

int main() {
    setenv("ROS_DOMAIN_ID", "10", 1);
    // Initialize micro-ROS transport
    rmw_uros_set_custom_transport(true, NULL, pico_serial_transport_open, pico_serial_transport_close,
                                  pico_serial_transport_write, pico_serial_transport_read);

    motor.Init();
    sleep_ms(100);
    odometry.Init(&motor);

    // Initialize LED
    cyw43_arch_init();
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    // Initialize micro-ROS components
    rcl_allocator_t allocator = rcl_get_default_allocator();
    // Initialize and modify options (Set DOMAIN ID to 10)
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, 10);

    // Initialize rclc support object with custom option
    rclc_support_t support;
    rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

    rcl_node_t node;
    rclc_node_init_default(&node, "cmd_vel_listener", "", &support);

    rclc_subscription_init_default(&cmd_vel_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                                   "/cmd_vel");
    rclc_publisher_init_default(&odometry_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
                                "/odom");
    rclc_publisher_init_default(&debug_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                                "/odom_debug");

    rclc_executor_t executor;
    rclc_executor_init(&executor, &support.context, 1, &allocator);

    // Add subscriber callback to executor
    rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &msg, cmd_vel_callback, ON_NEW_DATA);

    // Initialize odometry message
    odometry_msg.header.frame_id.data = strdup("odom");
    odometry_msg.child_frame_id.data = strdup("base_link");
    odometry_msg.header.frame_id.capacity = strlen(odometry_msg.header.frame_id.data) + 1;
    odometry_msg.child_frame_id.capacity = strlen(odometry_msg.child_frame_id.data) + 1;
    odometry.SetPublisher(&debug_publisher);

    while (true) {
        odometry.UpdateOdometry();
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        odometry.CalculateOdometry(odometry_msg);
        rcl_time_point_value_t now;
        rcutils_time_point_value_t time_now;
        rcutils_system_time_now(&time_now);
        now = static_cast<rcl_time_point_value_t>(time_now);
        // Populate the timestamp
        odometry_msg.header.stamp.sec = static_cast<uint32_t>(now / RCL_S_TO_NS(1));
        odometry_msg.header.stamp.nanosec = static_cast<uint32_t>(now % RCL_S_TO_NS(1));
        // Publish odometry data
        if (RCL_RET_OK != rcl_publish(&odometry_publisher, &odometry_msg, NULL)) {
            printf("Failed to publish odometry data\n");
        }
    }

    // Cleanup
    rcl_subscription_fini(&cmd_vel_subscriber, &node);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
    rclc_executor_fini(&executor);

    return 0;
}
