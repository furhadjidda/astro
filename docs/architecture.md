# Astro System Architecture

## Overview

Astro is a TurtleBot3-based differential-drive robot built around a **Raspberry Pi 4** running ROS 2 Humble. Low-level sensor reading and motor control run on microcontrollers (**Raspberry Pi Pico** or **Zephyr-based boards**) that communicate with the Pi 4 over micro-ROS (serial or UDP).

![System Architecture](https://www.plantuml.com/plantuml/svg/dLNHRk8u57tdAwuwBrHRjX1WXQhHQb92MZGKeWJQtMgbo2G6B1mxiXscp6Yb-OZzWlsq-PAvJYWjb9rTBJpWt9npVOxlzUM3DaIP8kEWjzhGB2OgMJ534rCeMjDh9dAYI0Q9p78fg32HsN8A2W54BFbho3L9raib2v7s9PSAtlck_Rv2k4h9JxkNtVBpwcN9JAGIL46lLjll06SUtL0kymntXrkf3TIVUh-Sdm7Hu0tkuTsWvtTy3xxKu4Kcd8NtxBC3b-3BfnikDxJ4X_V3nqDOD7Bl43Kc6wT4sESOqoBAWk5ze_7hFWXN44uYSC3p8gXxCYDCmD23LlEyzkU1vf3eV4wLsaB0e0FmxUjV0BDv8Km1ZlDpiuDxLgbkYspEAtN1i5EwyGQzOpTApgM9vug9PP7_8UgS529PLOTm7Gx7DxFWa42qKJBMd6I8xcEDvLAHV4NL-TxcAO8W6xOaXac1D7ie5NpitX-Y3ULKMY5hkfrBej8ADnsC1fCZ9HbBCB6IEYPB39Iukz0z16dEMKfKB6HACMiOZEmZkAsxtcM_LIKV3JqtF3eMIhYcGbCdJaX65K7kKA0uy3hTRaKBeqDEIdEp8gnKb2gseGf94_TZmuCHTlwkiTpXgGBWRJ4qpeivPtfL4cUXvp_lSxe4A4zQfA8Df6UvXk7TxF9c74MNCfNPhXw3SJG9J_7JhI0PUw8yjcXgr1Pp_JRgj9ixfR-FZvNgsB3iK6OBrhUV8U4oMU_KJeEtjB90yITYaefLhbAvt57SXnzrUS0I2LWvfzdPIGk6_KaBvu3lzmQjuuxs5QNX5A_0Jr1sI0ElGydomsaBeK8Qd3DikJ8W5yz1W34sYBugbz--_Wrp8PlTBjJnIC_tmPGj5bLfePv98vN6JPvFxmMs55hZ0nLfkTc_UdCUIdljxYjxpl-nvvoovvooLpMFBlrTWEsP2vXA_m8jwfoJxIdF_spfaUQhhO9hUr_j9NC_jbijEoC7KT1s6b4RbsvA5gfW1dnAp7vSF0u2DDgta_ROw2xfIQ-FDlh6x7CKmCtpDsRB4R2FbVbiQnv4vxoWHagpUk6cE01K6Y-EAsgdTZd3xN2zWWVMy9brRBq9Qcesl7jlc_KAPj4rAb6CS8JDt71gVxpW4EZy5w0zDFW1iFO1XU9VR-qx)

> Source: [docs/diagrams/system_architecture.puml](diagrams/system_architecture.puml)

## Data Flow — Key ROS 2 Topics

![Data Flow](https://www.plantuml.com/plantuml/svg/VLJRJXin47ttLupW1IIYG4kaY8UAa4tQg86W3KYjX6HvToTX5Q-ziXqW_O1-G3-nNzAndSkgRBlxClOvPyxO7llQEc7Skf9G22VuGkfNPbUbgeKH5UIwghL2vUPk8n4CvawefSG6vKRagwNHQrKCjTG6ZiS3_pSO0sFqwnRiXQy11cMkZK83VSRsXd1qM-P6vrevewL4ywHEAZ_JIMVpqoCG5cw7Z-Gs7lT75qsTiAJW5fNL9l06SpWUzSV9gD-aMIdBGXYkT86KExsV-c7CF9qaWpI8BiT9KsHGI9yPF_1SL6W4AIVAeOJaijUBudJUfYomTi-Yv4PRNfZo9IneDlZQIM1QEepIP31fqyOLwK9NwCo6kmNFZ1P5JlCOb_WmvZV3jbLIQQXhli9DfeKf8daq7StkQUk6tkgzKR5HeYhVKEujOt7VxzeS_89SMI6lrvaixVEselJXVizc_gIWq_daZo9yLz3Db3xlzNXPhRaHhwqKIgZV-59P-q-Ahp20OUula79tGrV_Xz0wxorR0M_7M6mF3yN88pONYh5qfqaFcjWetT00JsetpFrzE0nz9SmVVr27OEieLbspp8Zf8t_WgFUv_5IjI-KiEvGRU-5GRXmta2iAhi1ZyFldByY4HIvBjM9isncX13hjM7vLy1UKB4mSa7r-E46Lqnqo7P-eEALDeAEFc-23oDnr2PnSmEhB3s8fxH0oxHobq0lQP0QGOeUUewgsuBdRK2j29hUFP_3vRZu_0w4As3Kc8SIpOeaM5eOoxPy8XaJp7k-Sg7wI9VuEKbFlUXs-JIy_deD5yw8juTvbrz4Mjaiy-1woNrD8poVx0m==)

> Source: [docs/diagrams/data_flow.puml](diagrams/data_flow.puml)

## TF Tree

![TF Tree](https://www.plantuml.com/plantuml/svg/bPDHQuCm58NVyoj2-zo7mGQ-R5IhWR5AXwNqKQ96QvlaoasabB7_lfYEAXk3Jb-ydfplwidDqbYEjbUIsZgtA0GnfrPt7BcY9QWEjD1sQozIK1IbvRgHqoKHBqyDGg-h5KX0EcVXS4zMX8Xm_XQV_3KHlRC4r09fG0WKHzU3pXJlPfGRceRlT9u4x975DmqgK5xSn9lKgt4Iq0z2QTSNiZK7XtgcA_TJq23lXjArJjuO-Rmn2cv4Bbjzyg1e_IU66ukG3os5nKlk8YeGU5Mwzm9_0ci0ss6hxPNAN1YC1Sc3frhasjI0ob5TvBOj9_PMzUFdlAQx-vtIDKPlMuVR9BlVM78ba8zTLs5wl_by44BcKjHs1EnTYcmrRDOl_owB6Js2IyQZXFc5mhOvJ5dPlrXac1EwMBpucI1-C8OgMXEtjrSbOkEpItWiV6vZwN8OY_q4iCy8MGfTkQD2lW0=)

> Source: [docs/diagrams/tf_tree.puml](diagrams/tf_tree.puml)

## Zephyr micro-ROS Node Deep Dive

For implementation-level diagrams of `app_microros_node` (component, deployment, class, sequence, and activity), see:

- [micro-ROS Node Deep Dive](microros-node-deep-dive.md)

## Repository Layout

```
astro/
├── firmware/
│   ├── pico/               Pico firmware (FreeRTOS + micro-ROS)
│   │   ├── pico_*_node/    Pico application nodes
│   │   ├── sensor_drivers/ Sensor driver libraries
│   │   ├── tests_common/   Host-side shared test utilities
│   │   ├── free_rtos_config/ FreeRTOS configuration
│   │   ├── designs/        PlantUML design diagrams
│   │   └── external/       Git submodules (pico-sdk, FreeRTOS, micro-ROS, etc.)
│   │
│   └── zephyr/             Zephyr firmware (Zephyr RTOS + micro-ROS)
│       ├── app_*/          Zephyr application targets
│       ├── astro_custom_modules/ Astro-specific Zephyr modules
│       └── west/           West manifest (west.yml)
│
├── ros2/                   ROS 2 workspace for Raspberry Pi 4
│   ├── src/                ROS 2 package sources (directory names)
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
| ROS 2 (Pi4 cross) | Docker + Colcon | `python3 ros2/deploy/build_and_deploy_pi4.py --pi user@<ip>` | `ros2/install_pi4` |

## CI Pipelines

| Workflow | Trigger | What it builds |
|----------|---------|---------------|
| `main.yml` | push, PR | Pico firmware (pico_w + pico2_w) |
| `ros2.yml` | push, PR | ROS 2 workspace in `ros:humble` container |
| `zephyr.yml` | push, PR | Zephyr apps for ESP32-S3 boards (matrix) |
| `pi4-cross-compile.yml` | manual | ARM64 cross-compile via Docker + QEMU |
| `nightly.yml` | cron 22:15 UTC | Runs all of the above |
