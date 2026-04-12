import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    local_share = get_package_share_directory("astro_depthai_test")
    driver_pkg = "depthai_ros_driver_v3"
    driver_share = get_package_share_directory(driver_pkg)

    name = LaunchConfiguration("name", default="oak")
    camera_model = LaunchConfiguration("camera_model", default="OAK-D")
    parent_frame = LaunchConfiguration("parent_frame", default="oak_parent_frame")
    cam_pos_x = LaunchConfiguration("cam_pos_x", default="0.0")
    cam_pos_y = LaunchConfiguration("cam_pos_y", default="0.0")
    cam_pos_z = LaunchConfiguration("cam_pos_z", default="0.0")
    cam_roll = LaunchConfiguration("cam_roll", default="0.0")
    cam_pitch = LaunchConfiguration("cam_pitch", default="0.0")
    cam_yaw = LaunchConfiguration("cam_yaw", default="0.0")
    rs_compat = LaunchConfiguration("rs_compat", default="False")
    params_file = LaunchConfiguration(
        "params_file",
        default=os.path.join(local_share, "config", "rgbd_smoke.yaml"),
    )

    # Keep RViz separate: this smoke launch intentionally disables RViz.
    smoke_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(driver_share, "launch", "rgbd_pcl.launch.py")
        ),
        launch_arguments={
            "name": name,
            "camera_model": camera_model,
            "parent_frame": parent_frame,
            "cam_pos_x": cam_pos_x,
            "cam_pos_y": cam_pos_y,
            "cam_pos_z": cam_pos_z,
            "cam_roll": cam_roll,
            "cam_pitch": cam_pitch,
            "cam_yaw": cam_yaw,
            "params_file": params_file,
            "use_rviz": "False",
            "rs_compat": rs_compat,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("name", default_value=name),
            DeclareLaunchArgument("camera_model", default_value=camera_model),
            DeclareLaunchArgument("parent_frame", default_value=parent_frame),
            DeclareLaunchArgument("cam_pos_x", default_value=cam_pos_x),
            DeclareLaunchArgument("cam_pos_y", default_value=cam_pos_y),
            DeclareLaunchArgument("cam_pos_z", default_value=cam_pos_z),
            DeclareLaunchArgument("cam_roll", default_value=cam_roll),
            DeclareLaunchArgument("cam_pitch", default_value=cam_pitch),
            DeclareLaunchArgument("cam_yaw", default_value=cam_yaw),
            DeclareLaunchArgument("params_file", default_value=params_file),
            DeclareLaunchArgument("rs_compat", default_value=rs_compat),
            smoke_launch,
        ]
    )
