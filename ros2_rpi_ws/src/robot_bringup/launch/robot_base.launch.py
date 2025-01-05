from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    microros_node = Node(
        package="micro_ros_agent",
        executable = "micro_ros_agent",
        name='micro_ros_agent',
        arguments=["serial", "--dev", "/dev/ttyACM0"],
    )
    odometry_tf2_broadcaster = Node(
        package="astro_odometry_tf_broadcaster",
        executable = "odometry_tf_broadcaster",
    )
    base_link_to_base_footprint = Node(package = "tf2_ros",
                executable = "static_transform_publisher",
                arguments = ["0", "0", "0", "0", "0", "0", "base_link", "base_footprint"]
    )
    base_link_to_imu_link = Node(package = "tf2_ros",
                executable = "static_transform_publisher",
                arguments = ["0", "0", "0", "0", "0", "0", "base_link", "imu_link"]
    )
    base_link_to_laser_link = Node(package = "tf2_ros",
                executable = "static_transform_publisher",
                arguments = ["0", "0", "0", "0", "0", "0", "base_link", "laser"]
    )
    odom_to_base_link = Node(package = "tf2_ros",
                executable = "static_transform_publisher",
                arguments = ["0", "0", "0", "0", "0", "0", "odom", "base_link"]
    )
    ld.add_action(microros_node)
    ld.add_action(odometry_tf2_broadcaster)
    ld.add_action(base_link_to_base_footprint)
    ld.add_action(base_link_to_imu_link)
    ld.add_action(base_link_to_laser_link)
    ld.add_action(odom_to_base_link)
    return ld