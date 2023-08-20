from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    microros_node = Node(
        package="micro_ros_agent",
        executable = "micro_ros_agent",
        arguments=["serial", "--dev", "/dev/ttyACM0", "-v6"]
    )
    ld.add_action(microros_node)
    return ld