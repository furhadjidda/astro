# astro_robot_description

URDF robot description package for Astro.

## Contents

- **URDF models**: `urdf/burger.urdf`, `urdf/waffle.urdf`
- **3D meshes**: Bases, wheels, and sensor housings (RPLidar, RealSense, Hokuyo, etc.)
- **Launch file**: Starts `robot_state_publisher` + `joint_state_publisher`

## Usage

Automatically launched by `robot_bringup`. To launch standalone:

```bash
ros2 launch astro_robot_description display.launch.py
```

## Published Topics

- `/robot_description` — URDF XML string
- `/joint_states` — Joint positions
- TF tree from URDF joints
