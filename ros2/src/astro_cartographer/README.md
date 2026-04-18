# astro_cartographer

Standalone Cartographer SLAM launch (without IMU filter).

## Launch

```bash
ros2 launch astro_cartographer cartographer.launch.py
```

## Nodes Launched

- `cartographer_node` — 2D SLAM using `astro_lds_2d.lua` config
- `cartographer_occupancy_grid_node` — Occupancy grid publisher
- `rviz2` — Visualization
