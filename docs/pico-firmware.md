# Pico Firmware

Firmware for Raspberry Pi Pico (RP2040) and Pico 2W (RP2350) running FreeRTOS + micro-ROS.

## Pico Nodes

| Node | Directory | Description |
|------|-----------|-------------|
| `pico_bno055_node` | `firmware/pico/pico_bno055_node/` | Reads BNO055 IMU over I2C, publishes `sensor_msgs/Imu` via micro-ROS |
| `pico_differential_drive_node` | `firmware/pico/pico_differential_drive_node/` | PID-controlled DC motor drive with encoder odometry. Subscribes to `cmd_vel`, publishes `nav_msgs/Odometry` |
| `pico_gnss_node` | `firmware/pico/pico_gnss_node/` | Reads NMEA from PA1010D GPS, parses GGA sentences, publishes `sensor_msgs/NavSatFix` |
| `pico_sensors_node` | `firmware/pico/pico_sensors_node/` | Combined sensor node (IMU, GNSS, ToF, OLED display) using a sensor factory pattern |
| `pico_micro_ros_example_app` | `firmware/pico/pico_micro_ros_example_app/` | Minimal micro-ROS example for testing connectivity |

## Sensor Drivers

Reusable driver libraries in `firmware/pico/sensor_drivers/`:

- `adafruit_bno055` — 9-DOF IMU
- `adafruit_pa1010d_mini_gps` — GNSS module
- `adafruit_vl53l0x` — Time-of-Flight distance sensor
- `oled_display` — SSD1306 OLED
- `pca9685` — PWM driver for DC motors
- `flash_manager` — Flash storage utilities

## Building

Run these commands from the repository root. The `just` recipes select the correct SDK paths and target configuration.

### Host build (for running unit tests)

```bash
just pico-test
```

### Target build — Pico W (RP2040, Cortex-M0)

```bash
just pico pico_w
```

### Target build — Pico 2W (RP2350, Cortex-M33)

```bash
just pico pico2_w
```

### Flashing

1. Hold BOOTSEL and plug in the Pico (it appears as a USB mass storage device).
2. Copy the `.uf2` file from the build directory to the Pico drive. Or use:
   ```bash
   python3 tools/pico_reboot_and_copy/pico_reboot_and_copy.py firmware/pico/build_pico_w
   ```

See [Build, Flash, and Use Astro](build-flash-use.md) for the service-assisted flashing prerequisites.

## Design Documents

PlantUML diagrams in `firmware/pico/designs/`:

- `median_filter.puml` — Median filter for noisy sensor readings
- `node_creation.puml` — micro-ROS node initialization flow
- `odometry.puml` — Odometry computation
- `pid.puml` — PID controller design
- `rate_controller.puml` — FreeRTOS task rate control

## Dependencies (Git Submodules)

All under `firmware/pico/external/`:

- `pico-sdk` — Raspberry Pi Pico SDK
- `FreeRTOS-Kernel` — FreeRTOS
- `micro_ros_raspberrypi_pico_sdk` — micro-ROS for Pico
- `pimoroni-pico` — Pimoroni board support (Pico 2W)
- `googletest` — Google Test (host builds only)
