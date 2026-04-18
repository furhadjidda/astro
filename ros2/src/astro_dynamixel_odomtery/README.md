# astro_dynamixel_odomtery

Dynamixel XL430-W250-T servo driver with differential-drive odometry.

Controls two Dynamixel servos (left ID=1, right ID=2) via the Dynamixel SDK over `/dev/dynamixel_u2d2`. Reads encoder positions at 10 Hz and computes diff-drive odometry.

## Node: `odometry_node`

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `cmd_vel` | `geometry_msgs/Twist` | Velocity commands |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `odom` | `nav_msgs/Odometry` | Encoder-based odometry |

### TF Broadcast

`odom → base_link`

### Parameters

Configured via `param/dynamixel.yaml`.
