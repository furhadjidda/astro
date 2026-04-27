# robot_bringup

Main launch package for Astro. Orchestrates all hardware drivers and transforms.

## Launch

```bash
ros2 launch robot_bringup robot_base.launch.py \
    enable_lidar:=true \
    enable_microros:=true \
    enable_realsense:=false \
    enable_dynamixel:=false \
    microros_dev:=/dev/pimoroni_pico_2W
```

## Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `enable_microros` | `true` | Start micro-ROS agent |
| `enable_lidar` | `true` | Start RPLidar driver |
| `enable_realsense` | `true` | Start RealSense D455 camera |
| `enable_dynamixel` | `true` | Start Dynamixel servo driver with odometry |
| `microros_transport` | `serial` | micro-ROS transport: `serial` or `udp` |
| `microros_dev` | `/dev/ttyACM0` | Serial device for micro-ROS agent |
| `microros_udp_port` | `8888` | UDP port (when transport=udp) |

## Nodes Launched

- `micro_ros_agent` — Bridge between micro-ROS (Pico/Zephyr) and ROS 2
- `astro_odometry_tf_broadcaster` — Publishes `odom → base_link` TF from odometry
- `astro_dynamixel_odometry` — Dynamixel servo driver (when enabled)
- `sllidar_node` — RPLidar driver (when enabled)
- `robot_state_publisher` + `joint_state_publisher` — URDF-based TF tree
- `realsense2_camera` — Intel RealSense D455 (when enabled)
- `ros_time_publisher` — 1 kHz clock for micro-ROS sync
- `astro_sensor` — Sensor timestamp synchronization
- Static TF publishers: `base_link → base_footprint`, `imu_link`, `laser`, `camera_link`
