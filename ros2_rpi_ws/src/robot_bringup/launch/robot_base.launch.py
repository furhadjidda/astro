from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    channel_type =  LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')

    ld = LaunchDescription()
    DeclareLaunchArgument('scan_mode', default_value='Standard', description='Scan mode to be used by the LIDAR')

    DeclareLaunchArgument(
    'channel_type',
    default_value=channel_type,
    description='Specifying channel type of lidar')

    DeclareLaunchArgument(
        'serial_port',
        default_value=serial_port,
        description='Specifying usb port to connected lidar')

    DeclareLaunchArgument(
        'serial_baudrate',
        default_value=serial_baudrate,
        description='Specifying usb port baudrate to connected lidar')

    DeclareLaunchArgument(
        'frame_id',
        default_value=frame_id,
        description='Specifying frame_id of lidar')

    DeclareLaunchArgument(
        'inverted',
        default_value=inverted,
        description='Specifying whether or not to invert scan data')

    DeclareLaunchArgument(
        'angle_compensate',
        default_value=angle_compensate,
        description='Specifying whether or not to enable angle_compensate of scan data')

    DeclareLaunchArgument(
        'scan_mode',
        default_value=scan_mode,
        description='Specifying scan mode of lidar')

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
    astro_odom_publisher = Node(
            package="astro_odometry",
            executable = "astro_odometry",
            name='astro_odometry',
    )
    lidar_node = Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'channel_type':channel_type,
                         'serial_port': serial_port,
                         'serial_baudrate': serial_baudrate,
                         'frame_id': frame_id,
                         'inverted': inverted,
                         'angle_compensate': angle_compensate}],
            output='screen',
    )
    ld.add_action(microros_node)
    ld.add_action(odometry_tf2_broadcaster)
    ld.add_action(base_link_to_base_footprint)
    ld.add_action(base_link_to_imu_link)
    ld.add_action(base_link_to_laser_link)
    ld.add_action(odom_to_base_link)
    ld.add_action(astro_odom_publisher)
    ld.add_action(lidar_node)
    return ld