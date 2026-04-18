# ros_time_publisher

Publishes the current ROS clock as a time reference for micro-ROS nodes.

## Node: `ros_time_publisher`

Publishes at 1000 Hz on `/ros_time` so micro-ROS nodes (which lack an NTP-synced clock) can synchronize their message timestamps.

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/ros_time` | `builtin_interfaces/Time` | Current ROS clock at 1 kHz |
