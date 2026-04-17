import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_rviz = os.path.join(
        get_package_share_directory("astro_depthai_test"),
        "rviz",
        "depthai_pointcloud.rviz",
    )

    rviz_config = LaunchConfiguration("rviz_config", default=default_rviz)

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz_config",
                default_value=rviz_config,
                description="Path to RViz config file",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["--display-config", rviz_config],
            ),
        ]
    )
