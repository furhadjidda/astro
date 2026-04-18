# astro_teleop_twist_joy

Keyboard teleoperation node for Astro.

## Node: `teleop_keyboard`

Reads WASD/X keyboard input and publishes velocity commands with ramped profiles. Supports Burger and Waffle velocity limits via `TURTLEBOT3_MODEL` environment variable.

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `cmd_vel` | `geometry_msgs/Twist` | Velocity commands |

### Controls

| Key | Action |
|-----|--------|
| W | Forward |
| A | Turn left |
| S | Backward |
| D | Turn right |
| X | Stop |
