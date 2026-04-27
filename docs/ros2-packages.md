# ROS 2 Packages

ROS 2 Humble packages that run on the Raspberry Pi 4 (or development host).

## Packages

| Source Directory | ROS Package Name | Type | Description |
|------------------|------------------|------|-------------|
| `robot_bringup` | `robot_bringup` | Launch | Main launch file — brings up hardware drivers and transforms |
| `astro_robot_description` | `astro_robot_description` | Description | URDF models (burger/waffle), meshes, and state publishers |
| `astro_sensor_publisher` | `astro_sensor` | Python | Re-stamps raw micro-ROS sensor messages with synchronized ROS time |
| `astro_odometry_tf_broadcaster` | `astro_odometry_tf_broadcaster` | C++ | Subscribes to `/odom` and broadcasts the `odom -> base_link` TF |
| `astro_dynamixel_odomtery` | `astro_dynamixel_odometry` | C++ | Dynamixel XL430 servo driver with diff-drive odometry |
| `astro_slam` | `astro_slam` | Launch | Cartographer SLAM + Madgwick IMU filter |
| `astro_cartographer` | `astro_cartographer` | Launch | Standalone Cartographer launch (without IMU filter) |
| `astro_navigation` | `astro_navigation2` | Launch | Nav2 bringup with pre-built map |
| `astro_teleop_twist_joy` | `astro_teleop` | Python | Teleoperation publisher (`cmd_vel`) |
| `astro_depthai_test` | `astro_depthai_test` | Launch | OAK-D Lite RGBD smoke test and RViz launcher |
| `astro_ros_agent_watcher` | `ros_agent_watcher` | Python | micro-ROS agent watchdog (auto-restart on heartbeat timeout) |
| `ros_time_publisher` | `ros_time` | Python | Publishes ROS clock on `/ros_time` for micro-ROS sync |

See each package's own README for topics, parameters, and usage details.

## Third-Party Dependencies

Defined in `ros2/astro.repos` and pulled via `vcs import`:

| Package | Source | Sensor |
|---------|--------|--------|
| DynamixelSDK | ROBOTIS-GIT (humble branch) | DYNAMIXEL XL430-W250-T |
| realsense-ros | IntelRealSense (r/4.56.4) | Intel RealSense D455 |
| sllidar_ros2 | Slamtec (main) | RPLidar A1M8 / C1 |

DepthAI ROS (`ros-humble-depthai-ros-v3`) is installed from apt.

## Building

### Native build (on Pi4 or host)

```bash
cd ros2
vcs import src < astro.repos
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

### Cross-compile for Pi4 (from x86 host)

```bash
cd ros2/deploy
python3 build_and_deploy_pi4.py --pi ubuntu@<PI_IP> --first-run
```

See [ros2/deploy/README.md](../ros2/deploy/README.md) for Docker + QEMU setup details.

## Launching

The primary entry point is `robot_bringup`:

```bash
ros2 launch robot_bringup robot_base.launch.py \
    enable_realsense:=false \
    enable_lidar:=true \
    enable_microros:=true \
    enable_dynamixel:=false \
    microros_dev:=/dev/pimoroni_pico_2W
```

### Launch arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `enable_microros` | `true` | Start micro-ROS agent |
| `enable_lidar` | `true` | Start RPLidar driver |
| `enable_realsense` | `true` | Start RealSense D455 |
| `enable_dynamixel` | `true` | Start Dynamixel odometry driver |
| `microros_transport` | `serial` | micro-ROS transport: `serial` or `udp` |
| `microros_dev` | `/dev/ttyACM0` | Serial device for micro-ROS agent |
| `microros_udp_port` | `8888` | UDP port (when transport=udp) |

### Other launch files

```bash
# SLAM (Cartographer + IMU filter)
ros2 launch astro_slam astro_slam.launch.py

# Standalone cartographer
ros2 launch astro_cartographer cartographer.launch.py

# Navigation
ros2 launch astro_navigation2 navigation2.launch.py

# GNSS visualization
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

## Host ↔ Pi4 Network Setup

Both machines must be on the same network and share ROS domain settings:

```bash
# Add to ~/.bashrc on BOTH host and Pi4
export ROS_DOMAIN_ID=10
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Verify with:
```bash
# On Pi4:
ros2 run demo_nodes_cpp talker
# On host:
ros2 run demo_nodes_cpp listener
```

## Accessing Pi4 via VNC

```bash
# On Pi4:
vncserver :1 -localhost no -geometry 1920x1080

# Connect from host VNC client:
<pi-ip-address>:1
```
