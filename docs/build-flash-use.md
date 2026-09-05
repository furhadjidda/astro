# Build, Flash, and Use Astro

This guide uses the root [`justfile`](../justfile) as the canonical command interface. Run every `just` command from the repository root.

## Prerequisites

1. Clone the repository and initialize all firmware dependencies:
   ```bash
   git clone git@github.com:furhadjidda/astro.git
   cd astro
   git submodule update --init --recursive
   ```
2. Install [just](https://github.com/casey/just#installation), CMake, and the toolchain required by the target you intend to build.
3. For ROS 2, install ROS 2 Humble and source `/opt/ros/humble/setup.bash`.
4. For Zephyr, install the Zephyr SDK and `west` as described in the [Zephyr Getting Started Guide](https://docs.zephyrproject.org/latest/develop/getting_started/index.html).

List every supported build and cleanup recipe with:

```bash
just
```

## Pico Firmware

Build for a Pico W (RP2040) or Pico 2 W (RP2350):

```bash
just pico pico_w
just pico pico2_w
```

Build both targets or run the host unit tests:

```bash
just pico-all
just pico-test
```

### Flash a Pico

For a fresh or non-running Pico, hold **BOOTSEL** while connecting USB, then copy the required `.uf2` file from the relevant build directory to the mounted Pico volume.

For a running micro-ROS Pico, source the ROS 2 and workspace environments, then use the interactive uploader:

```bash
source /opt/ros/humble/setup.bash
source ros2/install/setup.bash
python3 tools/pico_reboot_and_copy/pico_reboot_and_copy.py firmware/pico/build_pico2_w
```

The uploader invokes the Pico's `reboot_to_bootloader` service, waits for its UF2 volume under `/media/$USER`, and presents the available firmware images. It requires `python3-tk` and a Pico firmware that provides that service; BOOTSEL remains the fallback.

## Zephyr Firmware

Initialize the west workspace once after cloning or after removing it:

```bash
just zephyr-init
```

Build an application for `rak` (RAK3112) or `feather` (Adafruit Feather ESP32-S3):

```bash
just zephyr app_sensor_node rak
just zephyr app_microros_node feather
just zephyr-examples rak
```

`just zephyr` supplies the target, board overlay, board configuration, and sysbuild settings. Build all supported application/board pairs only when needed because it is a lengthy operation:

```bash
just zephyr-all
```

### Flash an ESP32-S3

The RAK micro-ROS workflow builds and flashes in one step. Its default udev device is `/dev/rak_wireless_esp32s3`; provide a different device as the argument when needed:

```bash
just zephyr-microros-rak
just zephyr-microros-rak /dev/ttyUSB0
```

For any other completed Zephyr build, flash from `firmware/zephyr/` and identify its build directory:

```bash
cd firmware/zephyr
west flash --esp-device /dev/<device-symlink> --build-dir build/<app>
```

Install the repository's udev rules before relying on device symlinks. See [Hardware Setup](hardware-setup.md).

## ROS 2

Import source dependencies and install ROS dependencies once (or when `astro.repos` changes):

```bash
source /opt/ros/humble/setup.bash
just ros2-deps
```

Build the native workspace:

```bash
just ros2
source ros2/install/setup.bash
```

### Build and Deploy for Raspberry Pi 4

Set up ARM64 Docker emulation once on the x86 development host:

```bash
docker run --privileged --rm tonistiigi/binfmt --install arm64
```

Run the first cross-build and deploy, an ordinary deploy, or a build without deployment:

```bash
just ros2-pi4 ubuntu@<pi-ip> --first-run
just ros2-pi4 ubuntu@<pi-ip>
just ros2-pi4
```

Extra arguments are forwarded to the deployment script. For example, force a clean builder image on a build-only run:

```bash
just ros2-pi4 '' --clean --rebuild-image --first-run
```

See [ROS 2 Packages](ros2-packages.md) for all deployment options and launch files.

## Run the Robot

On the Pi, source the ROS installation and the workspace that was built or deployed, then launch the required hardware drivers:

```bash
source /opt/ros/humble/setup.bash
source ~/astro/ros2_ws/install/setup.bash
ros2 launch robot_bringup robot_base.launch.py \
    enable_lidar:=true \
    enable_microros:=true \
    enable_realsense:=false \
    enable_dynamixel:=false \
    microros_dev:=/dev/pimoroni_pico_2W
```

Use a udev symlink for `microros_dev` rather than a transient `/dev/ttyACM*` or `/dev/ttyUSB*` path. For UDP firmware, specify `microros_transport:=udp` and use `microros_udp_port:=8888` unless the firmware configuration uses another port.

Common additional workflows:

```bash
ros2 launch astro_slam astro_slam.launch.py
ros2 launch astro_navigation2 navigation2.launch.py
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

## Clean Build Outputs

The cleanup commands prompt before deletion:

```bash
just clean-pico
just clean-zephyr
just clean-zephyr-microros
just clean-ros2
just clean-ros2-pi4
just clean
```

Use `just clean-zephyr-all` when changing a Zephyr board or micro-ROS configuration, then rerun `just zephyr-init` if the west workspace itself was removed.