from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    channel_type = LaunchConfiguration("channel_type", default="serial")
    serial_port = LaunchConfiguration("serial_port", default="/dev/rplidar")
    serial_baudrate = LaunchConfiguration("serial_baudrate", default="460800")
    frame_id = LaunchConfiguration("frame_id", default="laser")
    inverted = LaunchConfiguration("inverted", default="false")
    angle_compensate = LaunchConfiguration("angle_compensate", default="true")
    scan_mode = LaunchConfiguration("scan_mode", default="Standard")
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    urdf_file_name = "urdf/waffle.urdf"
    urdf = os.path.join(
        get_package_share_directory("astro_robot_description"), urdf_file_name
    )
    with open(urdf, "r") as infp:
        robot_desc = infp.read()

    # Define launch arguments for each node (true/false)
    node_configs = {
        "enable_microros": "true",
        "enable_lidar": "true",
        "enable_realsense": "true",
        "enable_dynamixel": "true",
    }

    ld = LaunchDescription()

    # Declare launch arguments for each node
    for arg_name, default_value in node_configs.items():
        ld.add_action(DeclareLaunchArgument(arg_name, default_value=default_value))

    # Helper function to get IfCondition
    def is_enabled(node_name):
        return IfCondition(LaunchConfiguration(node_name))

    # Define launch arguments for lidar node
    lidar_args = [
        ("channel_type", channel_type, "Specifying channel type of lidar"),
        ("serial_port", serial_port, "Specifying usb port to connected lidar"),
        ("serial_baudrate", serial_baudrate, "Specifying baudrate"),
        ("frame_id", frame_id, "Specifying frame_id of lidar"),
        ("inverted", inverted, "Specifying whether or not to invert scan data"),
        ("angle_compensate", angle_compensate, "enable/disable angle_compensate"),
        ("scan_mode", scan_mode, "Specifying scan mode of lidar"),
    ]

    for arg_name, default_value, description in lidar_args:
        ld.add_action(
            DeclareLaunchArgument(
                arg_name, default_value=default_value, description=description
            )
        )
    DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    # Define nodes
    microros_node = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name="micro_ros_agent",
        arguments=["serial", "--dev", "/dev/ttyACM0"],
        condition=is_enabled("enable_microros"),
    )
    odometry_tf2_broadcaster = Node(
        package="astro_odometry_tf_broadcaster",
        executable="odometry_tf_broadcaster",
    )
    base_link_to_base_footprint = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "base_footprint"],
    )
    base_link_to_imu_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "imu_link"],
    )
    base_link_to_laser_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "laser"],
    )
    base_link_to_camera_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "camera_link"],
    )
    astro_odom_publisher = Node(
        package="astro_dynamixel_odometry",
        executable="astro_ros_odometry",
        name="astro_ros_odometry",
        condition=is_enabled("enable_dynamixel"),
    )
    lidar_node = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar_node",
        parameters=[
            {
                "channel_type": channel_type,
                "serial_port": serial_port,
                "serial_baudrate": serial_baudrate,
                "frame_id": frame_id,
                "inverted": inverted,
                "angle_compensate": angle_compensate,
                "scan_mode": scan_mode,
            }
        ],
        output="screen",
        condition=is_enabled("enable_lidar"),
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "robot_description": robot_desc}],
        arguments=[urdf],
    )
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_gui": use_sim_time}],
    )

    # Path to RealSense launch file
    realsense_launch_file = os.path.join(
        get_package_share_directory("realsense2_camera"), "launch", "rs_launch.py"
    )

    # Check if the launch file exists
    if not os.path.exists(realsense_launch_file):
        raise FileNotFoundError(
            f"RealSense launch file not found: {realsense_launch_file}"
        )

    # Include RealSense launch file with parameters
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_file),
        launch_arguments={
            "enable_gyro": "false",
            "enable_accel": "false",
            "rgb_camera.color_profile": "640x480x15",
            "initial_reset": "true",
            "rgb_camera.power_line_frequency": "1",
            # "depth_module.depth_profile": "640x480x15",
            "enable_depth": "false",
            "enable_infra1": "false",
            "enable_infra2": "false",
        }.items(),
        condition=is_enabled("enable_realsense"),
    )

    ros_time_publisher = Node(
        package="ros_time", executable="ros_time", name="ros_time", output="screen"
    )

    astro_sensor_publisher = Node(
        package="astro_sensor",
        executable="astro_sensor",
        name="astro_sensor",
        output="screen",
    )

    ld.add_action(realsense_launch)
    ld.add_action(microros_node)
    ld.add_action(odometry_tf2_broadcaster)
    ld.add_action(base_link_to_base_footprint)
    ld.add_action(base_link_to_imu_link)
    ld.add_action(base_link_to_laser_link)
    ld.add_action(base_link_to_camera_link)
    ld.add_action(astro_odom_publisher)
    ld.add_action(lidar_node)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(ros_time_publisher)
    ld.add_action(astro_sensor_publisher)

    return ld
