# astro_depthai_test

Smoke-test launchers for the OAK-D Lite (DepthAI) camera.

## Launch Files

- `rgb_stereo_smoke.launch.py` — Starts OAK-D driver with RGBD + point cloud (headless)
- `rviz_depthai.launch.py` — Opens RViz with a DepthAI point cloud visualization config

## Usage

```bash
ros2 launch astro_depthai_test rgb_stereo_smoke.launch.py
ros2 launch astro_depthai_test rviz_depthai.launch.py
```
