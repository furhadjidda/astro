#include <math.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/string_functions.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <sensor_msgs/msg/nav_sat_status.h>
#include <std_msgs/msg/string.h>

#include <Adafruit_GPS.hpp>
#include <memory>
#include <string>

#include "bno055.hpp"
#include "gnss_parser.hpp"
#include "hardware/i2c.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "pico_uart_transports.h"
#include "rcl/time.h"
#include "rcutils/time.h"

#define PUBLISH_RATE_HZ 5
#define GPSECHO false
#define GPS_I2C_ADDRESS 0x10

// GNSS portion
// GPS parser and device
std::unique_ptr<gnss_parser> parser = std::make_unique<gnss_parser>();
std::unique_ptr<Adafruit_GPS> GPS = std::make_unique<Adafruit_GPS>(i2c0);
// GNSS Message
sensor_msgs__msg__NavSatFix nav_sat_fix_msg;
// Debug Message
std_msgs__msg__String dbg_msg;

// ROS Publishers
rcl_publisher_t gnss_publisher;

// IMU portion
// Structure for quaternion
struct Quaternion {
    float x;
    float y;
    float z;
    float w;
};

bno055_sensor::Bno055 imu;
// IMU message
sensor_msgs__msg__Imu imu_msg;

// ROS Publishers
rcl_publisher_t imu_publisher;
rcl_publisher_t dbg_publisher;

// Timer
uint32_t timer = millis();

// Initialize IMU
bool init_bno055() {
    if (!imu.initialization()) {
        printf("BNO055 initialization failed!\n");
        return false;
    }
    printf("BNO055 initialized successfully.\n");
    return true;
}

// Publish Debug data
void publish_debug_message(const std::string &message) {
    // Ensure the message memory is managed correctly
    if (!rosidl_runtime_c__String__assign(&dbg_msg.data, message.c_str())) {
        // Handle error
        printf("Failed to assign debug message\n");
        return;
    }

    // Publish the debug message
    if (RCL_RET_OK != rcl_publish(&dbg_publisher, &dbg_msg, NULL)) {
        printf("Failed to publish debug message\n");
    }
}

// Initialize GNSS
void setup_gps() {
    if (!GPS->Init(GPS_I2C_ADDRESS)) {
        publish_debug_message("Error initializing GPS\n");
        return;
    }
    // Configure GPS
    GPS->SendCommand(reinterpret_cast<const uint8_t *>(PMTK_SET_NMEA_OUTPUT_ALLDATA),
                     strlen(PMTK_SET_NMEA_OUTPUT_ALLDATA));
    GPS->SendCommand(reinterpret_cast<const uint8_t *>(PMTK_SET_NMEA_UPDATE_1HZ), strlen(PMTK_SET_NMEA_UPDATE_1HZ));
    GPS->SendCommand(reinterpret_cast<const uint8_t *>(PGCMD_ANTENNA), strlen(PGCMD_ANTENNA));
    sleep_ms(1000);
}

// populate IMU message
void populate_imu_msg(sensor_msgs__msg__Imu &msg) {
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
}

// process GNSS data and populate GNSS message
void process_gps_data() {
    char c = GPS->ReadData();

    if (GPS->NewNMEAreceived()) {
        std::string nmea_sentence(GPS->LastNMEA());
        if (!GPS->Parse(GPS->LastNMEA())) {
            return;
        }

        // Publish debug message
        std::string fix_status = (GPS->mFix) ? "true" : "false";
        publish_debug_message("Received NMEA sentence: " + nmea_sentence + " >>> Fix: " + fix_status);
        // Pack and publish GNSS data
        nav_sat_fix_msg = parser->packData(GPS->mLatitude, GPS->mLat, GPS->mLongitude, GPS->mLon, GPS->mAltitude,
                                           GPS->mFix, GPS->mFixquality_3d, GPS->mHDOP, GPS->mPDOP, GPS->mVDOP);
        rcl_time_point_value_t now;
        rcutils_time_point_value_t time_now;
        rcutils_system_time_now(&time_now);
        now = static_cast<rcl_time_point_value_t>(time_now);
        // Populate the timestamp
        nav_sat_fix_msg.header.stamp.sec = static_cast<uint32_t>(now / RCL_S_TO_NS(1));
        nav_sat_fix_msg.header.stamp.nanosec = static_cast<uint32_t>(now % RCL_S_TO_NS(1));
        rcl_publish(&gnss_publisher, &nav_sat_fix_msg, NULL);
    }
    if (millis() - timer > 2000) {
        timer = millis();
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, !cyw43_arch_gpio_get(CYW43_WL_GPIO_LED_PIN));
    }
}
int main() {
    setenv("ROS_DOMAIN_ID", "10", 1);
    rmw_uros_set_custom_transport(true, NULL, pico_serial_transport_open, pico_serial_transport_close,
                                  pico_serial_transport_write, pico_serial_transport_read);

    // Initialize BNO055
    if (!init_bno055()) {
        return 1;
    }
    // Initialize GPS
    setup_gps();

    // This is for LED on pico
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
    // initialize Node
    rclc_node_init_default(&node, "imu_gnss_publisher", "", &support);
    // initialize publishers
    rclc_publisher_init_default(&imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data");
    rclc_publisher_init_default(&gnss_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix),
                                "gnss_data");
    rclc_publisher_init_default(&dbg_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "dbg_msg");

    rcl_timer_t timer;
    // initialize timers
    rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(1000 / PUBLISH_RATE_HZ),
                            [](rcl_timer_t *timer, int64_t last_call_time) {
                                populate_imu_msg(imu_msg);
                                rcl_publish(&imu_publisher, &imu_msg, NULL);
                                cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, !cyw43_arch_gpio_get(CYW43_WL_GPIO_LED_PIN));
                            });

    rclc_executor_t executor;
    // initialize executor
    rclc_executor_init(&executor, &support.context, 1, &allocator);

    // add the timer to the executor
    rclc_executor_add_timer(&executor, &timer);
    nav_sat_fix_msg.status.service = sensor_msgs__msg__NavSatStatus__SERVICE_GPS;

    printf("Starting micro-ROS gnss_publisher...\n");
    while (rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)) == RCL_RET_OK) {
        // Main loop spins the executor
        process_gps_data();
    }

    // Clean up
    rcl_publisher_fini(&imu_publisher, &node);
    rclc_support_fini(&support);
    rcl_publisher_fini(&gnss_publisher, &node);
    rcl_publisher_fini(&dbg_publisher, &node);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
    rclc_executor_fini(&executor);

    return 0;
}
