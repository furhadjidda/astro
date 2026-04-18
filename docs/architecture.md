# Astro System Architecture

## Overview

Astro is a TurtleBot3-based differential-drive robot built around a **Raspberry Pi 4** running ROS 2 Humble. Low-level sensor reading and motor control run on microcontrollers (**Raspberry Pi Pico** or **Zephyr-based boards**) that communicate with the Pi 4 over micro-ROS (serial or UDP).

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        Development Host (x86)                          │
│                                                                        │
│   RViz2 / FoxGlove ◄──── ROS 2 DDS (Domain ID 10) ────►  SLAM /     │
│                            (same network)                  Nav2       │
└──────────────────────────────────┬──────────────────────────────────────┘
                                   │ Wi-Fi / Ethernet
                                   ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                     Raspberry Pi 4  (Ubuntu 22.04)                     │
│                          ROS 2 Humble                                  │
│                                                                        │
│  ┌──────────────┐  ┌──────────────┐  ┌───────────────┐  ┌──────────┐ │
│  │ robot_bringup│  │ astro_slam   │  │astro_navigation│  │ teleop   │ │
│  │  (launch)    │  │(Cartographer)│  │   (Nav2)       │  │(keyboard)│ │
│  └──────┬───────┘  └──────────────┘  └───────────────┘  └──────────┘ │
│         │                                                              │
│         ├──► micro_ros_agent ◄── serial/UDP ──► Microcontrollers      │
│         ├──► sllidar_node (RPLidar A1M8 / C1)                        │
│         ├──► realsense2_camera (Intel D455)                           │
│         ├──► depthai_ros_driver (OAK-D Lite)                          │
│         ├──► robot_state_publisher (URDF: burger / waffle)            │
│         ├──► astro_sensor (re-stamps IMU, GNSS, odom)                │
│         ├──► astro_dynamixel_odometry (Dynamixel XL430 drive)        │
│         ├──► ros_time_publisher (1 kHz clock for micro-ROS sync)     │
│         └──► ros_agent_watcher (micro-ROS watchdog)                  │
│                                                                        │
└──────────────────────────────┬─────────────────────────────────────────┘
                               │ USB serial / UART / UDP
            ┌──────────────────┼──────────────────┐
            ▼                  ▼                  ▼
┌───────────────────┐ ┌────────────────┐ ┌────────────────────────┐
│  Raspberry Pi     │ │  Raspberry Pi  │ │  Zephyr Board          │
│  Pico (RP2040)    │ │  Pico 2W       │ │  (RAK3112/ESP32-S3     │
│                   │ │  (RP2350)      │ │   Adafruit Feather)    │
│  FreeRTOS +       │ │  FreeRTOS +    │ │                        │
│  micro-ROS        │ │  micro-ROS     │ │  Zephyr RTOS +         │
│                   │ │                │ │  micro-ROS             │
│  Nodes:           │ │  Nodes:        │ │                        │
│  • bno055 (IMU)   │ │  • sensors     │ │  Apps:                 │
│  • diff_drive     │ │    (IMU, GNSS, │ │  • sensor_node         │
│    (motors +      │ │     ToF,       │ │  • microros_node       │
│     odometry)     │ │     display)   │ │  • bluetooth           │
│  • gnss           │ │  • diff_drive  │ │  • sd_card_fs          │
│                   │ │                │ │                        │
└───────────────────┘ └────────────────┘ └────────────────────────┘
         │                    │                    │
         ▼                    ▼                    ▼
    ┌─────────┐        ┌───────────┐        ┌───────────┐
    │ Sensors │        │  Sensors  │        │  Sensors  │
    │ BNO055  │        │  BNO055   │        │  IIM42652 │
    │ PCA9685 │        │  PA1010D  │        │  PA1010D  │
    │ Encoders│        │  VL53L0X  │        │  SSD1306  │
    └─────────┘        │  SSD1306  │        └───────────┘
                       └───────────┘
```

## Data Flow — Key ROS 2 Topics

```
Microcontroller ──micro-ROS──►  /bno055_imu_raw   ──► astro_sensor ──► /imu/data_raw
                                                                    ──► /imu
                                /ublox_gnss_raw    ──► astro_sensor ──► /gnss
                                /odom_raw          ──► astro_sensor ──► /odom

RPLidar ──USB──► sllidar_node ──► /scan

Intel D455 ──USB──► realsense2_camera ──► /camera/depth, /camera/color, /camera/imu

OAK-D Lite ──USB──► depthai_ros_driver ──► /oak/rgb, /oak/stereo/depth, /oak/points

                                /odom ──► astro_odometry_tf_broadcaster ──► TF: odom → base_link
                                /cmd_vel ◄── teleop_keyboard
                                /cmd_vel ──► astro_dynamixel_odometry ──► Dynamixel servos
                                                                      ──► /odom (encoder-based)

ros_time_publisher ──► /ros_time (1 kHz clock reference for micro-ROS nodes)
```

## TF Tree

```
map
 └── odom                          (published by Cartographer or EKF)
      └── base_link                (published by astro_odometry_tf_broadcaster)
           ├── base_footprint      (static)
           ├── imu_link            (static)
           ├── laser               (static)
           ├── camera_link         (static)
           ├── wheel_left_link     (from URDF joint states)
           └── wheel_right_link    (from URDF joint states)
```

## Repository Layout

```
astro/
├── firmware/
│   ├── pico/               Pico firmware (FreeRTOS + micro-ROS)
│   │   ├── nodes/          Pico application nodes
│   │   ├── drivers/        Sensor driver libraries
│   │   ├── common/         Shared utilities (median filter, etc.)
│   │   ├── config/         FreeRTOS config
│   │   ├── designs/        PlantUML design diagrams
│   │   └── external/       Git submodules (pico-sdk, FreeRTOS, micro-ROS, etc.)
│   │
│   └── zephyr/             Zephyr firmware (Zephyr RTOS + micro-ROS)
│       ├── apps/           Zephyr application targets
│       ├── custom_modules/ Astro-specific Zephyr modules
│       └── west/           West manifest (west.yml)
│
├── ros2/                   ROS 2 workspace for Raspberry Pi 4
│   ├── src/                ROS 2 packages (see per-package READMEs)
│   ├── deploy/             Cross-compile & deploy tooling for Pi4
│   └── astro.repos         VCS import file for third-party packages
│
├── tools/                  Development host utilities
├── config/
│   ├── udev/               udev rules for USB devices
│   └── rviz/               RViz2 layout configs
│
├── docker/                 Dockerfiles (Zephyr CI, Pi4 cross-compile)
└── docs/                   Project documentation & images
```

## Hardware Configurations

Astro supports two drive systems that can be swapped:

| Config | Motors | Controller | Odometry Source |
|--------|--------|-----------|-----------------|
| **DC Motor** (Burger style) | 12V 60RPM gearmotor w/ encoder | Waveshare DC Motor Driver on Pico | Encoder ticks via `pico_differential_drive_node` |
| **Dynamixel** (Waffle style) | DYNAMIXEL XL430-W250-T | U2D2 Power Hub | Dynamixel SDK position feedback via `astro_dynamixel_odometry` |

## Build Targets

| Target | Build System | Command | Output |
|--------|-------------|---------|--------|
| Pico (RP2040) | CMake | `cmake -S . -B build -DPICO_BOARD=pico_w -DMCU_TYPE=cortex-m0 && cmake --build build` | `.uf2` files |
| Pico 2W (RP2350) | CMake | `cmake -S . -B build -DPICO_BOARD=pico2_w -DPICO_PLATFORM=rp2350 -DMCU_TYPE=cortex-m33 && cmake --build build` | `.uf2` files |
| Pico host tests | CMake + GTest | `cmake -S . -B build -DBUILD_FOR_HOST=ON && cmake --build build && ctest --test-dir build` | Test binaries |
| Zephyr (ESP32-S3) | West | `west build -p always -b <board> --sysbuild app_sensor_node -- -DDTC_OVERLAY_FILE=... -DEXTRA_CONF_FILE=...` | `.bin` / flashable |
| ROS 2 (native) | Colcon | `cd ros2/ && colcon build` | Install tree |
| ROS 2 (Pi4 cross) | Docker + Colcon | `python3 ros2/deploy/build_and_deploy_pi4.py --pi user@<ip>` | ARM64 install tree |

## CI Pipelines

| Workflow | Trigger | What it builds |
|----------|---------|---------------|
| `main.yml` | push, PR | Pico firmware (pico_w + pico2_w) |
| `ros2.yml` | push, PR | ROS 2 workspace in `ros:humble` container |
| `zephyr.yml` | push, PR | Zephyr apps for ESP32-S3 boards (matrix) |
| `pi4-cross-compile.yml` | manual | ARM64 cross-compile via Docker + QEMU |
| `nightly.yml` | cron 22:15 UTC | Runs all of the above |
