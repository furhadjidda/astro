# Zephyr Firmware

Firmware for Zephyr RTOS-based boards (ESP32-S3, RAK Wireless, etc.) running micro-ROS.

## Applications

| App | Directory | Description |
|-----|-----------|-------------|
| `app_sensor_node` | `firmware/zephyr/app_sensor_node/` | Combined sensor node (IMU, GNSS, display) for Zephyr boards |
| `app_microros_node` | `firmware/zephyr/app_microros_node/` | micro-ROS node for sensor data publishing |
| `app_bluetooth` | `firmware/zephyr/app_bluetooth/` | BLE connectivity application |
| `app_sd_card_fs` | `firmware/zephyr/app_sd_card_fs/` | SD card filesystem for data logging |
| `app_examples` | `firmware/zephyr/app_examples/` | Example and test applications |

## Supported Boards

| Board | Target | Overlay | Config |
|-------|--------|---------|--------|
| Adafruit Feather ESP32-S3 | `adafruit_feather_esp32s3/esp32s3/procpu` | `boards/adafruit_feather_s3.overlay` | `boards/adafruit_feather_s3.conf` |
| RAK3112 (ESP32-S3) | `rak3112/esp32s3/procpu` | `boards/rak_wireless_rak3312.overlay` | `boards/rak_wireless_rak3312.conf` |
| ESP32-S3 DevKitC | `esp32s3_devkitc/esp32s3/procpu` | `boards/esp32s3_devkitc.overlay` | `boards/esp32_s3.conf` |
| Pimoroni Pico Plus 2 | `pico_plus2/rp2350b/m33` | `boards/pimoroni_pico_plus_2w.overlay` | `boards/pimoroni_pico_plus_2w.conf` |

## Prerequisites

Follow the [Zephyr Getting Started Guide](https://docs.zephyrproject.org/latest/develop/getting_started/index.html) to set up the toolchain.

Tested versions:
- CMake 3.22.1
- Python 3.10.12
- Zephyr 4.3.0 (via west manifest)
- Zephyr SDK 0.17.4
- West 1.5

## Building

Run these commands from the repository root. The `just` recipes provide Astro's board target, overlay, configuration, and sysbuild options.

### Initial workspace setup

```bash
just zephyr-init
```

### Build for a board

```bash
# RAK3112
just zephyr app_sensor_node rak

# Adafruit Feather ESP32-S3
just zephyr app_sensor_node feather
```

> **Important:** Clean micro-ROS artifacts before switching boards:
> ```bash
> rm -rf build/ modules/libmicroros/micro_ros_src/build \
>        modules/libmicroros/micro_ros_src/install/ \
>        modules/libmicroros/micro_ros_src/log/
> ```

### Flashing

```bash
# Build and flash the RAK micro-ROS application.
just zephyr-microros-rak

# Or flash an existing ESP32 build manually.
cd firmware/zephyr
west flash --esp-device /dev/<device_symlink> --build-dir build/<app>
```

See [Build, Flash, and Use Astro](build-flash-use.md) for supported recipe arguments and clean-build guidance.

## Debugging ESP32-S3 via JTAG

Terminal 1 — start OpenOCD:
```bash
openocd -f board/esp32s3-builtin.cfg
```

Terminal 2 — start GDB:
```bash
<Zephyr SDK path>/xtensa-espressif_esp32s3_zephyr-elf/bin/xtensa-espressif_esp32s3_zephyr-elf-gdb
(gdb) target extended-remote :3333
```

## Custom Modules

`firmware/zephyr/astro_custom_modules/` contains Astro-specific Zephyr modules:

- `gnss/` — GNSS driver integration
- `iim42652_rak12033/` — RAK12033 IMU driver (IIM-42652)
- `libmicroros/` — micro-ROS Zephyr module
- `sensor/` — Sensor abstraction layer

## References

Project structure adapted from [micro-ROS Zephyr module](https://github.com/micro-ROS/micro_ros_zephyr_module).
