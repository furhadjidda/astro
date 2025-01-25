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

#include <dynamixel_sdk/dynamixel_sdk.h>
#include <tf2_ros/transform_broadcaster.h>

#include <cmath>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

// Constants for Dynamixel and robot parameters
constexpr int DXL_ID_LEFT = 1;                 // Left motor ID
constexpr int DXL_ID_RIGHT = 2;                // Right motor ID
constexpr int BAUDRATE = 57600;                // Communication baud rate
constexpr char DEVICENAME[] = "/dev/ttyUSB0";  // Device name for the U2D2 port
constexpr float WHEEL_RADIUS = 0.033;          // Radius of the wheels in meters
constexpr float WHEEL_BASE = 0.160;            // Distance between the wheels in meters
constexpr int TICKS_PER_REV = 4096;            // Encoder ticks per revolution
constexpr float UPDATE_RATE = 10.0;            // Update rate in Hz

/**
 * @class OdometryNode
 * @brief A ROS2 node for handling odometry using Dynamixel motors.
 *
 * This class subscribes to velocity commands, controls Dynamixel motors, and publishes odometry information.
 *
 * @details
 * The OdometryNode class initializes the Dynamixel motors, sets them to velocity mode, and enables torque.
 * It subscribes to the "cmd_vel" topic to receive velocity commands and publishes odometry information
 * on the "odom" topic. The node also broadcasts the transform between the "odom" and "base_link" frames.
 *
 * The class handles the following tasks:
 * - Initializing and configuring Dynamixel motors.
 * - Subscribing to velocity commands.
 * - Calculating and publishing odometry information.
 * - Broadcasting the transform between "odom" and "base_link".
 *
 * @note
 * The node assumes the use of two Dynamixel motors for differential drive.
 *
 * @param DEVICENAME The name of the device port for Dynamixel communication.
 * @param BAUDRATE The baud rate for Dynamixel communication.
 * @param DXL_ID_LEFT The ID of the left Dynamixel motor.
 * @param DXL_ID_RIGHT The ID of the right Dynamixel motor.
 * @param WHEEL_BASE The distance between the two wheels.
 * @param WHEEL_RADIUS The radius of the wheels.
 * @param TICKS_PER_REV The number of encoder ticks per revolution of the wheel.
 * @param UPDATE_RATE The rate at which odometry is updated (in Hz).
 */
class OdometryNode : public rclcpp::Node {
   public:
    /**
     * @brief Constructor for the OdometryNode class.
     *
     * This constructor initializes the ROS 2 node, sets up publishers and subscribers,
     * initializes the Dynamixel motors, and configures them for velocity control mode.
     * It also sets up a timer to periodically update the odometry.
     *
     * @details
     * - Initializes the node with the name "odometry_node".
     * - Initializes member variables for position (x_, y_, theta_) and previous encoder ticks (prev_left_ticks_,
     * prev_right_ticks_).
     * - Creates a publisher for the "odom" topic to publish odometry messages.
     * - Creates a subscriber for the "cmd_vel" topic to receive velocity commands.
     * - Initializes the TransformBroadcaster for broadcasting TF transforms.
     * - Initializes the Dynamixel port handler and packet handler.
     * - Opens the port and sets the baud rate for communication with the Dynamixel motors.
     * - Configures the left and right motors to velocity mode.
     * - Enables torque for both motors.
     * - Sets up a timer to call the updateOdometry function at a specified update rate.
     */
    OdometryNode() : Node("odometry_node"), x_(0.0), y_(0.0), theta_(0.0), prev_left_ticks_(0), prev_right_ticks_(0) {
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&OdometryNode::cmdVelCallback, this, std::placeholders::_1));
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Dynamixel initialization
        port_handler_ = dynamixel::PortHandler::getPortHandler(DEVICENAME);
        packet_handler_ = dynamixel::PacketHandler::getPacketHandler(2.0);

        if (!port_handler_->openPort()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open the port");
            rclcpp::shutdown();
        }

        if (!port_handler_->setBaudRate(BAUDRATE)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set baud rate");
            rclcpp::shutdown();
        }

        // Configure motors to velocity mode
        configureMotorToVelocityMode(DXL_ID_LEFT);
        configureMotorToVelocityMode(DXL_ID_RIGHT);

        // Enable torque for both motors
        enableTorque(DXL_ID_LEFT);
        enableTorque(DXL_ID_RIGHT);

        // Timer to update odometry
        timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000 / UPDATE_RATE)),
                                         std::bind(&OdometryNode::updateOdometry, this));
    }

    /**
     * @brief Destructor for the OdometryNode class.
     *
     * This destructor is responsible for disabling the torque on the left and right Dynamixel motors
     * and closing the communication port. It ensures that the motors are properly shut down and the
     * port is closed when the OdometryNode object is destroyed.
     */
    ~OdometryNode() {
        // Disable torque and close the port
        disableTorque(DXL_ID_LEFT);
        disableTorque(DXL_ID_RIGHT);
        port_handler_->closePort();
    }

   private:
    /**
     * @brief Enables the torque for a specified motor.
     *
     * This function sends a command to enable the torque for the motor with the given ID.
     * It writes a value of 1 to the control table address 64 of the specified motor.
     *
     * @param motor_id The ID of the motor for which to enable torque.
     */
    void enableTorque(int motor_id) {
        int dxl_comm_result = packet_handler_->write1ByteTxRx(port_handler_, motor_id, 64, 1);
        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to enable torque for motor %d", motor_id);
        }
    }

    /**
     * @brief Disables the torque for a specified motor.
     *
     * This function sends a command to disable the torque for the motor with the given ID.
     *
     * @param motor_id The ID of the motor for which to disable the torque.
     */
    void disableTorque(int motor_id) { packet_handler_->write1ByteTxRx(port_handler_, motor_id, 64, 0); }

    /**
     * @brief Reads the current position (ticks) of the specified motor.
     *
     * This function communicates with the motor using the Dynamixel protocol to read
     * the current position value (in ticks) from the motor's control table.
     *
     * @param motor_id The ID of the motor to read the position from.
     * @return The current position of the motor in ticks. If the read operation fails,
     *         an error message is logged and the returned value may be invalid.
     */
    int readTicks(int motor_id) {
        uint32_t position = 0;
        int dxl_comm_result = packet_handler_->read4ByteTxRx(port_handler_, motor_id, 132, &position);
        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read position for motor %d", motor_id);
        }
        return static_cast<int>(position);
    }

    /**
     * @brief Calculate the difference between the current and previous tick counts,
     *        accounting for wrap-around in a circular encoder.
     *
     * This function computes the difference between the current and previous tick
     * counts of an encoder. If the difference exceeds half the number of ticks per
     * revolution (TICKS_PER_REV), it adjusts the difference to account for the
     * wrap-around effect of the circular encoder.
     *
     * @param current_ticks The current tick count from the encoder.
     * @param prev_ticks The previous tick count from the encoder.
     * @return The adjusted tick difference, accounting for wrap-around.
     */
    int calculateTickDifference(int current_ticks, int prev_ticks) {
        int diff = current_ticks - prev_ticks;
        if (diff > TICKS_PER_REV / 2) {
            diff -= TICKS_PER_REV;
        } else if (diff < -TICKS_PER_REV / 2) {
            diff += TICKS_PER_REV;
        }
        return diff;
    }

    /**
     * @brief Converts a yaw angle to a quaternion representation.
     *
     * This function takes a yaw angle (rotation around the Z-axis) and converts it
     * to a quaternion, which is a common representation for 3D rotations in robotics
     * and computer graphics.
     *
     * @param yaw The yaw angle in radians.
     * @return geometry_msgs::msg::Quaternion The quaternion representation of the yaw angle.
     */
    geometry_msgs::msg::Quaternion yawToQuaternion(float yaw) {
        geometry_msgs::msg::Quaternion q;
        q.x = 0.0;
        q.y = 0.0;
        q.z = std::sin(yaw / 2);
        q.w = std::cos(yaw / 2);
        return q;
    }

    /**
     * @brief Callback function to handle velocity commands.
     *
     * This function is called whenever a new velocity command is received.
     * It calculates the individual wheel velocities based on the linear and
     * angular velocities from the command message and sets the goal velocities
     * for the left and right motors.
     *
     * @param msg A shared pointer to the received Twist message containing
     *            the linear and angular velocity commands.
     */
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Calculate individual wheel velocities
        float linear_velocity = msg->linear.x;    // Forward/backward velocity
        float angular_velocity = msg->angular.z;  // Rotational velocity

        float left_wheel_velocity = linear_velocity - (WHEEL_BASE / 2.0) * angular_velocity;
        float right_wheel_velocity = linear_velocity + (WHEEL_BASE / 2.0) * angular_velocity;
        right_wheel_velocity *= -1;  // Reverse direction

        // Set goal velocities for the motors
        setGoalVelocity(DXL_ID_LEFT, left_wheel_velocity);
        setGoalVelocity(DXL_ID_RIGHT, right_wheel_velocity);
    }

    /**
     * @brief Set the goal velocity for a specified motor.
     *
     * This function converts the desired velocity in meters per second (m/s) to the
     * corresponding raw value required by the Dynamixel motor and sends the command
     * to set the motor's velocity.
     *
     * @param motor_id The ID of the motor to set the velocity for.
     * @param velocity The desired velocity in meters per second (m/s).
     */
    void setGoalVelocity(int motor_id, float velocity) {
        // Convert velocity (m/s) to Dynamixel raw value
        float rpm = velocity / (2 * M_PI * WHEEL_RADIUS) * 60;
        int raw_velocity = static_cast<int>(rpm / 0.229);  // 0.229 rpm per unit (XL430 spec)
        int dxl_comm_result = packet_handler_->write4ByteTxRx(port_handler_, motor_id, 104, raw_velocity);
        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set velocity for motor %d", motor_id);
        }
    }

    /**
     * @brief Configures the specified motor to operate in velocity mode.
     *
     * This function sends a command to the motor with the given ID to set it to velocity mode.
     * It uses the Dynamixel SDK to communicate with the motor and handle any communication errors.
     *
     * @param motor_id The ID of the motor to configure.
     */
    void configureMotorToVelocityMode(int motor_id) {
        uint8_t dxl_error = 0;
        int dxl_comm_result =
            packet_handler_->write1ByteTxRx(port_handler_, motor_id, 11, 1, &dxl_error);  // Velocity mode
        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set velocity mode for motor %d: %s", motor_id,
                         packet_handler_->getTxRxResult(dxl_comm_result));
        } else if (dxl_error != 0) {
            RCLCPP_ERROR(this->get_logger(), "Dynamixel error for motor %d: %s", motor_id,
                         packet_handler_->getRxPacketError(dxl_error));
        } else {
            RCLCPP_INFO(this->get_logger(), "Motor %d set to velocity mode", motor_id);
        }
    }

    /**
     * @brief Updates the odometry based on encoder values.
     *
     * This function reads the encoder values from the left and right wheels,
     * calculates the distance traveled by each wheel, and updates the robot's
     * position (x_, y_) and orientation (theta_). It then publishes the updated
     * odometry information.
     *
     * The function performs the following steps:
     * 1. Reads the current encoder values for the left and right wheels.
     * 2. Calculates the difference in encoder ticks since the last update.
     * 3. Converts the tick differences to distances traveled by each wheel.
     * 4. Computes the average distance traveled and the change in orientation.
     * 5. Updates the robot's position and orientation.
     * 6. Publishes the updated odometry information.
     *
     * The encoder values are read using the readTicks function, and the tick
     * differences are calculated using the calculateTickDifference function.
     * The distances are calculated based on the wheel radius and the number of
     * ticks per revolution. The change in orientation is calculated based on
     * the wheel base.
     *
     * @note The right wheel's tick difference is negated to account for the
     *       reverse direction of the right wheel.
     */
    void updateOdometry() {
        // Read encoder values
        int left_ticks = readTicks(DXL_ID_LEFT);
        int right_ticks = readTicks(DXL_ID_RIGHT);

        // Calculate distances
        int left_ticks_diff = calculateTickDifference(left_ticks, prev_left_ticks_);
        int right_ticks_diff = calculateTickDifference(right_ticks, prev_right_ticks_);
        right_ticks_diff = -right_ticks_diff;  // Reverse direction
        prev_left_ticks_ = left_ticks;
        prev_right_ticks_ = right_ticks;

        float left_distance = (2 * M_PI * WHEEL_RADIUS * left_ticks_diff) / TICKS_PER_REV;
        float right_distance = (2 * M_PI * WHEEL_RADIUS * right_ticks_diff) / TICKS_PER_REV;

        float distance = (left_distance + right_distance) / 2.0;
        float delta_theta = (right_distance - left_distance) / WHEEL_BASE;

        theta_ += delta_theta;
        x_ += distance * std::cos(theta_);
        y_ += distance * std::sin(theta_);

        // Publish odometry
        publishOdometry(distance, delta_theta);
    }

    /**
     * @brief Publishes the odometry information.
     *
     * This function creates and publishes an odometry message with the given distance and delta_theta.
     * The odometry message includes the position, orientation, and velocity of the robot.
     *
     * @param distance The distance traveled by the robot.
     * @param delta_theta The change in orientation of the robot.
     */
    void publishOdometry(float distance, float delta_theta) {
        auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
        odom_msg->header.stamp = this->get_clock()->now();
        odom_msg->header.frame_id = "odom";
        odom_msg->child_frame_id = "base_link";

        // Position
        odom_msg->pose.pose.position.x = x_;
        odom_msg->pose.pose.position.y = y_;
        odom_msg->pose.pose.position.z = 0.0;

        // Orientation
        odom_msg->pose.pose.orientation = yawToQuaternion(theta_);

        // Velocity (assuming constant time interval)
        odom_msg->twist.twist.linear.x = distance * UPDATE_RATE;
        odom_msg->twist.twist.angular.z = delta_theta * UPDATE_RATE;

        odom_publisher_->publish(std::move(odom_msg));
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    dynamixel::PortHandler *port_handler_;
    dynamixel::PacketHandler *packet_handler_;

    float x_, y_, theta_;
    int prev_left_ticks_, prev_right_ticks_;
};

// Main function to initialize and run the ROS 2 node
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdometryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
