/*
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 *  Description: Node that subscribe to odometry message publisher and broadcast its associated
 *  transform frame
 *  Author: Furhad Jidda
 *
 *  rosrun odometry_tf_broadcaster broadcast
 */
#include <functional>
#include <memory>
#include <sstream>
#include <string>


#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <nav_msgs/msg/odometry.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"


class FramePublisher : public rclcpp::Node
{
public:
    FramePublisher()
    : Node("odometry_tf_broadcaser")
    {
        // Initialize the transform broadcaster
        mTfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
        // callback function on each message
        mSubscription = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&FramePublisher::HandleOdometryMsg, this, std::placeholders::_1));
    }

private:
    void HandleOdometryMsg(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
    {
        geometry_msgs::msg::TransformStamped t;

        // Read message content and assign it to
        // corresponding tf variables
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = msg->header.frame_id;
        t.child_frame_id = msg->child_frame_id;

        // Turtle only exists in 2D, thus we get x and y translation
        // coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = msg->pose.pose.position.x;
        t.transform.translation.y = msg->pose.pose.position.y;
        t.transform.translation.z = msg->pose.pose.position.z;

        t.transform.rotation = msg->pose.pose.orientation;

        // Send the transformation
        mTfBroadcaster->sendTransform(t);
    }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mSubscription;
  std::unique_ptr<tf2_ros::TransformBroadcaster> mTfBroadcaster;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FramePublisher>());
  rclcpp::shutdown();
  return 0;
}