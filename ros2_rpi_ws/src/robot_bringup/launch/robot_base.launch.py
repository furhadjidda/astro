from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    microros_node = Node(
        package="micro_ros_agent",
        executable = "micro_ros_agent",
        arguments=["serial", "--dev", "/dev/ttyACM0", "-v6"]
    )
    odometry_tf2_broadcaster = Node(
        package="astro_odometry_tf_broadcaster",
        executable = "odometry_tf_broadcaster",
    )
    ld.add_action(microros_node)
    ld.add_action(odometry_tf2_broadcaster)
    return ld