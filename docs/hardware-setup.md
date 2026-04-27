# Hardware Setup

## udev Rules

udev rules live in `config/udev/` and must be copied to the Pi4:

```bash
sudo cp config/udev/*.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### Identifying device serial numbers

Each udev rule matches a specific USB device by serial number. To find yours:

```bash
udevadm info --name=/dev/ttyACM0 --attribute-walk | grep -i "ATTRS{serial}"
```

Update `99-custom.rules` with the correct `ATTRS{serial}` for your Pico boards.

### Device symlinks

After udev rules are installed, devices appear at predictable paths:

| Symlink | Device |
|---------|--------|
| `/dev/pimoroni_pico_2W` | Pimoroni Pico 2W |
| `/dev/pimoroni_pico_inventor` | Pimoroni Pico Inventor |
| `/dev/raspberry_pico` | Standard Raspberry Pi Pico |
| `/dev/rplidar` | RPLidar A1M8 / C1 |
| `/dev/dynamixel_u2d2` | U2D2 Dynamixel controller |
| `/dev/adafruit_esp32s3_zephyr` | Adafruit Feather ESP32-S3 |
| `/dev/espressif_esp32_s3` | ESP32-S3 DevKitC |

## RViz Configurations

Pre-configured RViz layouts in `config/rviz/`:

- `astro_all_sensors.rviz` — Full sensor visualization (lidar, camera, IMU, TF)

## Useful Commands

### Static transforms (manual, for debugging)

```bash
# Odometry
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 odom base_link

# IMU
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map imu_frame
```

### Launching lidar standalone

```bash
ros2 launch sllidar_ros2 view_sllidar_a1_launch.py scan_mode:=Standard
```

### Visualizing IMU in RViz

```bash
sudo apt install ros-$ROS_DISTRO-imu-tools
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map imu_frame
```
