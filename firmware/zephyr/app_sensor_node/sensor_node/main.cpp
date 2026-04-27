
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

#include <version.h>

#if ZEPHYR_VERSION_CODE >= ZEPHYR_VERSION(3, 1, 0)
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#else
#include <sys/printk.h>
#include <zephyr.h>
#endif

#include <microros_transports.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>
#if CONFIG_ZSN_STATUS_CLASS
#include "status_class.hpp"
#endif
#include <zephyr/device.h>

/* =========================================================
 * Configuration
 * ========================================================= */

#define EXECUTOR_STACK_SIZE 4096
#define TIME_SYNC_STACK_SIZE 1024

#define EXECUTOR_PRIORITY 5
#define TIME_SYNC_PRIORITY 7 /* lower priority */

#define PUBLISH_PERIOD_MS 1000
#define TIME_SYNC_PERIOD_MS 1000

/* =========================================================
 * Error handling macros
 * ========================================================= */

#define RCCHECK(fn)                                            \
    do {                                                       \
        rcl_ret_t rc = (fn);                                   \
        if (rc != RCL_RET_OK) {                                \
            printk("RCL error %d at line %d\n", rc, __LINE__); \
            for (;;) {                                         \
                k_sleep(K_FOREVER);                            \
            }                                                  \
        }                                                      \
    } while (0)

#define RCSOFTCHECK(fn)                                          \
    do {                                                         \
        rcl_ret_t rc = (fn);                                     \
        if (rc != RCL_RET_OK) {                                  \
            printk("RCL warning %d at line %d\n", rc, __LINE__); \
        }                                                        \
    } while (0)

/* =========================================================
 * micro-ROS objects
 * ========================================================= */

static rclc_support_t support;
static rcl_node_t node;
static rcl_publisher_t publisher;
static rcl_timer_t timer;
static rclc_executor_t executor;

static std_msgs__msg__Int32 msg;

/* =========================================================
 * Thread objects
 * ========================================================= */

K_THREAD_STACK_DEFINE(executor_stack, EXECUTOR_STACK_SIZE);
static struct k_thread executor_thread;

K_THREAD_STACK_DEFINE(time_sync_stack, TIME_SYNC_STACK_SIZE);
static struct k_thread time_sync_thread;

/* =========================================================
 * Timer callback (runs inside executor thread)
 * ========================================================= */

static void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
    ARG_UNUSED(last_call_time);

    if (timer != NULL) {
        RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
        msg.data++;
    }
}

/* =========================================================
 * micro-ROS executor thread
 * ========================================================= */

static void executor_thread_entry(void* a, void* b, void* c) {
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        k_sleep(K_MSEC(1));
    }
}

/* =========================================================
 * Time synchronization thread
 * ========================================================= */

static void time_sync_thread_entry(void* a, void* b, void* c) {
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);
#if CONFIG_ZSN_STATUS_CLASS
    StatusClass status_led;
    status_led.display_color(colors[0]);
#endif
    /* Give transport + agent time to come up */
    k_sleep(K_SECONDS(2));
    while (1) {
        bool ok = rmw_uros_sync_session(50); /* 50 ms timeout */

        if (!ok) {
            printk("micro-ROS time sync failed\n");
        }
        k_sleep(K_MSEC(TIME_SYNC_PERIOD_MS));
    }
}

/* =========================================================
 * main()
 * ========================================================= */

int main(void) {
    printk("Zephyr micro-ROS example starting\n");

    k_sleep(K_MSEC(10)); /* allow rail to stabilize */

    /* Allow system to stabilize */
    k_sleep(K_SECONDS(2));

    /* Configure custom transport */
    rmw_uros_set_custom_transport(true, NULL, zephyr_transport_open, zephyr_transport_close, zephyr_transport_write,
                                  zephyr_transport_read);

    /* Allocator */
    rcl_allocator_t allocator = rcl_get_default_allocator();

    /* Init options with custom domain ID */
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, 10));

    /* micro-ROS support */
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    /* Node */
    node = rcl_get_zero_initialized_node();
    RCCHECK(rclc_node_init_default(&node, "zephyr_publisher", "", &support));

    /* Publisher */
    RCCHECK(rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
                                        "/zephyr_int_publisher"));

    /* Timer */
    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(PUBLISH_PERIOD_MS), timer_callback));

    /* Executor */
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    msg.data = 0;

    /* Start executor thread */
    k_thread_create(&executor_thread, executor_stack, EXECUTOR_STACK_SIZE, executor_thread_entry, NULL, NULL, NULL,
                    EXECUTOR_PRIORITY, 0, K_NO_WAIT);

    k_thread_name_set(&executor_thread, "uros_executor");

    /* Start time sync thread */
    k_thread_create(&time_sync_thread, time_sync_stack, TIME_SYNC_STACK_SIZE, time_sync_thread_entry, NULL, NULL, NULL,
                    TIME_SYNC_PRIORITY, 0, K_NO_WAIT);

    k_thread_name_set(&time_sync_thread, "uros_time_sync");

    printk("micro-ROS threads started\n");

    /* main thread does nothing further */
    while (1) {
        k_sleep(K_FOREVER);
    }
}
