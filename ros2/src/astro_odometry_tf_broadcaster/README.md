# astro_odometry_tf_broadcaster

Subscribes to odometry messages and broadcasts the corresponding TF transform.

## Node: `odometry_tf_broadcaster`

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/odom` | `nav_msgs/Odometry` | Odometry with pose |

### TF Broadcast

Publishes `odom → base_link` transform extracted from the odometry message's pose, using the message's `frame_id` and `child_frame_id`.
