# Raspberry Pi 4 setup

The Pi needs a 64-bit OS and SSH access; ROS and build tools are not needed
on the host. The container runs privileged with host networking and access to
all host devices. Treat access to the container and the Docker group as root
access. **Before starting the robot, secure or lift the wheels:** the existing
Dynamixel driver configures velocity mode and enables motor torque on startup.

## Host setup

1. Install Docker Engine and the Compose plugin using the official instructions
   for the Pi's OS: [Ubuntu](https://docs.docker.com/engine/install/ubuntu/) or
   [Debian/Raspberry Pi OS](https://docs.docker.com/engine/install/debian/).
   Install the `docker-ce`, `docker-ce-cli`, `containerd.io`,
   `docker-buildx-plugin`, and `docker-compose-plugin` packages as directed.
   Verify `docker version` and `docker compose version`. Configure the SSH user
   to run Docker without interactive sudo, for example using Docker's
   [post-install steps](https://docs.docker.com/engine/install/linux-postinstall/).
   Log in again after changing group membership.
2. If the robot uses these interfaces, edit `/boot/firmware/config.txt` on
   Ubuntu 22.04 and enable only the required settings:

   ```ini
   dtparam=i2c_arm=on
   dtparam=spi=on
   enable_uart=1
   ```

   Reboot after changes. For UART, disable any serial login console that
   occupies the chosen port and check the Pi model's UART/overlay mapping.
   TODO(confirm): which interfaces and UART pins does this robot actually use?

## Persistent device names

On the Pi, use `ls -l /dev/serial/by-id/` and inspect each connected adapter:

```bash
udevadm info --query=property --name=/dev/ttyUSB0
udevadm info --attribute-walk --name=/dev/ttyUSB0
```

Repeat for the lidar and Dynamixel devices, substituting their current paths.
Find the vendor ID, product ID, and unique serial on the **same USB parent**.
Create `/etc/udev/rules.d/99-astro-serial.rules` with the following template,
replacing every placeholder with the measured values:

```udev
# TODO(confirm): replace placeholders with the Dynamixel USB parent attributes.
SUBSYSTEM=="tty", ATTRS{idVendor}=="DYNAMIXEL_VENDOR", ATTRS{idProduct}=="DYNAMIXEL_PRODUCT", ATTRS{serial}=="DYNAMIXEL_SERIAL", SYMLINK+="dynamixel dynamixel_u2d2"
# TODO(confirm): replace placeholders with the lidar USB parent attributes.
SUBSYSTEM=="tty", ATTRS{idVendor}=="LIDAR_VENDOR", ATTRS{idProduct}=="LIDAR_PRODUCT", ATTRS{serial}=="LIDAR_SERIAL", SYMLINK+="rplidar"
```

If an adapter lacks a unique serial, use its measured `ID_PATH` property
instead of `ATTRS{serial}` (and keep it in the same physical USB port); do not
match identical adapters by vendor/product alone. Check existing rules for
conflicting names. The driver hardcodes `/dev/dynamixel_u2d2`; the same rule
also creates the requested `/dev/dynamixel`. The lidar launch defaults to
`/dev/rplidar`.

Install the RealSense rules matching the SDK:

```bash
curl -fsSL https://raw.githubusercontent.com/IntelRealSense/librealsense/v2.56.4/config/99-realsense-libusb.rules \
  | sudo tee /etc/udev/rules.d/99-realsense-libusb.rules >/dev/null
```

For the DepthAI Movidius USB device, this repository's
[`config/udev/80-luxonis-movidius.rules`](../../config/udev/80-luxonis-movidius.rules)
contains the existing `03e7` rule. Copy it to `/etc/udev/rules.d/` on the Pi.
TODO(confirm): verify the connected OAK's booted and unbooted USB IDs with
`lsusb`; add the applicable vendor-recommended rules if those IDs differ.
The vendor rules may grant broad host access to USB devices.

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -l /dev/dynamixel /dev/dynamixel_u2d2 /dev/rplidar
```

Reconnect devices if needed. The container sees these host-created symlinks
through `/dev`; no per-device Docker mounts are required. The micro-ROS serial
device defaults to `/dev/ttyACM0`; TODO(confirm): set a stable path and pass
`microros_dev:=...` in the Compose command if that is not correct.

## Build and deploy

Run on the desktop from `ros2/`:

```bash
make setup-buildx                 # one-time, installs ARM64 binfmt on the host
make build                        # Dockerfile: DockerFile, target: linux/arm64
make deploy PI=pi.local PI_USER=ubuntu
make logs PI=pi.local PI_USER=ubuntu
make shell PI=pi.local PI_USER=ubuntu
```

`PI` can instead be `user@host` with `PI_USER=`. Override `IMAGE`,
`REMOTE_DIR`, `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, `BUILD_JOBS`, and
`PARALLEL_WORKERS` as make variables when needed. Use the same `IMAGE` and
`REMOTE_DIR` for later `logs` and `shell` commands. Defaults are domain 10 and
Fast DDS. The deploy target builds on the desktop, streams a compressed image
over SSH, copies Compose to `astro-robot/` in the SSH user's home, and starts
the service. Nothing is compiled on the Pi.

GitHub Actions runs the ARM64 image build and a hardware-free package and
launch-argument check on pushes to `main` and pull requests targeting `main`.
Successful main and eligible same-repository PR builds are published to GHCR;
fork and Dependabot PRs are build-only. See the [development guide](../README.md#optional-transfer-through-a-registry)
for tags and Pi usage. CI does not deploy to the Pi or verify hardware. Clean
QEMU builds can be slow and consume significant Actions minutes. The equivalent
local build is:

```bash
docker buildx build --platform linux/arm64 --load -f DockerFile \
  --build-arg BUILD_JOBS=2 --build-arg PARALLEL_WORKERS=2 \
  -t astro:humble-arm64 .
```

Base image digests and the librealsense version are pinned, but external Git
sources follow branches. A clean fetch can pick up new commits, so builds are
not fully reproducible; ROS/Ubuntu apt packages, rosdep rules and some upstream
build downloads are also not snapshot pinned. The external fetch and colcon
layers and BuildKit ccache are reused when local packages change. To fetch
current branch heads deliberately, rebuild the fetch stage on the desktop:

```bash
docker buildx build --platform linux/arm64 --load -f DockerFile \
  --no-cache-filter fetch -t astro:humble-arm64 .
```

Use the same image tag and build arguments you normally use. A normal
`make build` can reuse cached external sources even if a branch has advanced.
The DepthAI Humble apt packages require compiler and CMake packages even at
runtime; they remain installed to keep apt dependencies consistent, but no
compilation runs on the Pi.

Registry deployment is an alternative to `make ship`; it is not part of the
current deploy target.

## Pi smoke tests

On the desktop, after deployment, open a shell in the running container:

```bash
make shell PI=pi.local PI_USER=ubuntu
```

In that container shell:

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 pkg list | grep -E 'robot_bringup|dynamixel|sllidar|realsense|depthai|micro_ros_agent'
rs-enumerate-devices
ros2 topic list
ls -l /dev/dynamixel /dev/dynamixel_u2d2 /dev/rplidar
timeout 15s ros2 topic echo --once --qos-reliability best_effort /odom
timeout 15s ros2 topic echo --once --qos-reliability best_effort /scan
```

On the desktop, `make logs PI=pi.local PI_USER=ubuntu` follows the logs; Ctrl+C
stops following without stopping the robot. Check for Dynamixel errors and
motor mode messages; confirm `/odom` and `/scan` contain live data. These
commands do not independently verify motor IDs or physically command motion.
Device ownership/baud rate and the lidar scan mode must match the hardware.

DepthAI is **not** included in `robot_base.launch.py`. Start its existing
smoke launch separately, while no other process owns that camera:

```bash
ros2 launch astro_depthai_test rgb_stereo_smoke.launch.py
```

Check the output and `ros2 topic list` in another shell. TODO(confirm): OAK
model and launch parameters. The micro-ROS device and RealSense configuration
also require connected, compatible hardware; an image build alone cannot
verify them.

For service status, common deployment errors, and further diagnostics, see
[TROUBLESHOOTING.md](TROUBLESHOOTING.md).
