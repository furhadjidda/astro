# astro_slam

SLAM launch package combining Google Cartographer with Madgwick IMU filter.

## Launch

```bash
ros2 launch astro_slam slam.launch.py
```

## Nodes Launched

- `cartographer_node` — 2D SLAM from laser scan + IMU
- `cartographer_occupancy_grid_node` — Generates occupancy grid from Cartographer map
- `imu_filter_madgwick_node` — Fuses raw IMU data into orientation estimates
- `rviz2` — Visualization

## Configuration

- Cartographer Lua config in `config/`
- IMU filter params in `config/`
- EKF config for `robot_localization` in `config/`
