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

#include "MotorControl.hpp"
#include "micro_ros_arduino.h"
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include <stdio.h>
#include "Odometry.hpp"

rcl_subscription_t subscriber;
rcl_publisher_t publisher;
rcl_publisher_t logPublisher;
std_msgs__msg__Int32 msg;
std_msgs__msg__String gString;
rclc_executor_t executor;
rclc_executor_t pubexecutor;
rclc_executor_t logpubexecutor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

Odometry odometry;
MotorControl motor;

#define RCCHECK(fn)                    \
    {                                  \
        rcl_ret_t temp_rc = fn;        \
        if ((temp_rc != RCL_RET_OK)) { \
            error_loop();              \
        }                              \
    }
#define RCSOFTCHECK(fn)                \
    {                                  \
        rcl_ret_t temp_rc = fn;        \
        if ((temp_rc != RCL_RET_OK)) { \
        }                              \
    }



void error_loop()
{
    while (1) {
        delay(100);
    }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    //odometry.UpdateOdometry();
    //nav_msgs__msg__Odometry odometryData;
    //odometry.CalculateOdometry( odometryData );
    //rcl_publish(&publisher, &odometryData, NULL);
  }
}

void cmd_vel_callback(const void* msgin)
{
    odometry.cmd_vel_callback(msgin);
}

void setup()
{
    set_microros_transports();

    motor.Init();

    allocator = rcl_get_default_allocator();

    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

    // create subscriber
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel"));

    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &cmd_vel_callback, ON_NEW_DATA));


    // TODO add also a publisher
    // create publisher
    RCCHECK(rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
      "odom"));

    RCCHECK(rclc_publisher_init_default(
      &logPublisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "debug"));

    // create timer,
    const unsigned int timer_timeout = 10;
    RCCHECK(rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      timer_callback));
    RCCHECK(rclc_executor_init(&pubexecutor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_init(&logpubexecutor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&pubexecutor, &timer));
    msg.data = 0;

    odometry.Init( &motor );
    odometry.SetPublisher(&logPublisher);
}

void loop()
{
    odometry.UpdateOdometry();
    nav_msgs__msg__Odometry odometryData;
    odometry.CalculateOdometry( odometryData );
    rcl_publish(&publisher, &odometryData, NULL);
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
    RCCHECK(rclc_executor_spin_some(&pubexecutor, RCL_MS_TO_NS(100)));
    RCCHECK(rclc_executor_spin_some(&logpubexecutor, RCL_MS_TO_NS(100)));
}
