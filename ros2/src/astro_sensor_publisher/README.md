# astro_sensor_publisher

Sensor synchronization node that re-stamps raw micro-ROS messages with the current ROS clock time and applies EMA smoothing to IMU data.

## Node: `sensor_sync_node`

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/bno055_imu_raw` | `sensor_msgs/Imu` | Raw IMU from micro-ROS |
| `/ublox_gnss_raw` | `sensor_msgs/NavSatFix` | Raw GNSS from micro-ROS |
| `/odom_raw` | `nav_msgs/Odometry` | Raw odometry from micro-ROS |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/imu/data_raw` | `sensor_msgs/Imu` | Time-synced raw IMU |
| `/imu` | `sensor_msgs/Imu` | EMA-smoothed IMU (α=0.1) |
| `/gnss` | `sensor_msgs/NavSatFix` | Time-synced GNSS fix |
| `/odom` | `nav_msgs/Odometry` | Time-synced odometry |
