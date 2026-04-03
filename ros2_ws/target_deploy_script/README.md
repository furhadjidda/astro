# Build ROS 2 for Raspberry Pi 4 on Host

This folder contains a host-side pipeline that builds your ROS 2 workspace for Raspberry Pi 4 (ARM64), then deploys runtime files to the Pi so you can run without building on the robot.

The implementation is in `build_and_deploy_pi4.py` and is aligned with your CI flow in `.github/workflows/ros2.yml`.

## What it does

1. Builds an ARM64 Docker image with ROS tools.
2. Runs `colcon build` in ARM64 mode on your host.
3. Produces a Pi-specific install tree: `install_pi4/`.
4. Syncs `install_pi4/` (and runtime assets) to your Pi over `rsync`.

## Prerequisites

- Docker with `buildx` support.
- QEMU binfmt for ARM64 emulation (`linux/arm64` container execution).
- SSH access from host to Pi.
- Same ROS distro installed on Pi (default in scripts: `humble`).

## One-time host setup

```bash
# Install QEMU emulation handlers for cross-architecture containers
docker run --privileged --rm tonistiigi/binfmt --install all

# Optional: verify buildx is available
docker buildx version
```

## Build + deploy

From `ros2_ws`:

```bash
./target_deploy_script/build_and_deploy_pi4.py --quick --pi ubuntu@astro-pi --target "robot_bringup"

# One-time setup: build deps image and exit
./target_deploy_script/build_and_deploy_pi4.py --init

# Full refresh when environment or dependencies changed
./target_deploy_script/build_and_deploy_pi4.py --full --pi ubuntu@astro-pi

# If this is your first run on this host, auto-setup ARM64 emulation
./target_deploy_script/build_and_deploy_pi4.py --setup-binfmt --quick --pi ubuntu@astro-pi
```

## Simple Modes

Use these for most workflows:

```bash
# One-time prep
--init

# Fast daily loop
--quick

# Full rebuild/refresh
--full

# Package targeting shortcut
--target "pkg_a pkg_b"

```

Detailed/advanced examples:

```bash
./target_deploy_script/build_and_deploy_pi4.py --pi ubuntu@astro-pi

# One-time: bake rosdeps into a dedicated image for faster daily builds
./target_deploy_script/build_and_deploy_pi4.py --prepare-deps-image

# Daily: use deps image and build/deploy your packages
./target_deploy_script/build_and_deploy_pi4.py --deps-image --pi ubuntu@astro-pi
```

Example with package subset:

```bash
./target_deploy_script/build_and_deploy_pi4.py \
  --pi ubuntu@192.168.1.50 \
  --packages-up-to "robot_bringup astro_sensor_publisher"

# Build only the package(s) you changed (fast iteration)
./target_deploy_script/build_and_deploy_pi4.py \
  --pi ubuntu@192.168.1.50 \
  --packages-select "astro_dynamixel_odomtery"

# Skip known non-critical packages and keep building others
./target_deploy_script/build_and_deploy_pi4.py \
  --pi ubuntu@192.168.1.50 \
  --packages-skip "dynamixel_sdk_examples" \
  --continue-on-error

# Ignore known unresolved rosdep keys and continue
./target_deploy_script/build_and_deploy_pi4.py \
  --pi ubuntu@192.168.1.50 \
  --rosdep-skip-keys "micro_ros_agent" \
  --rosdep-continue-on-error

# Force refresh rosdep cache (usually not needed on every run)
./target_deploy_script/build_and_deploy_pi4.py \
  --pi ubuntu@192.168.1.50 \
  --force-rosdep-update

# Optionally import external repos from astro.repos before building
./target_deploy_script/build_and_deploy_pi4.py \
  --pi ubuntu@192.168.1.50 \
  --import-repos

# If CMake package exports are stale/inconsistent, clean caches once
./target_deploy_script/build_and_deploy_pi4.py \
  --pi ubuntu@192.168.1.50 \
  --clean

# Faster alternative: clean only one problematic package and force CMake cache refresh
./target_deploy_script/build_and_deploy_pi4.py \
  --pi ubuntu@192.168.1.50 \
  --clean-packages "depthai_ros_driver" \
  --cmake-clean-cache

# If OpenSSL detection still fails, force explicit CMake hints (defaults already set in script)
./target_deploy_script/build_and_deploy_pi4.py \
  --pi ubuntu@192.168.1.50 \
  --packages-select "depthai_ros_driver" \
  --clean-packages "depthai_ros_driver" \
  --cmake-clean-cache \
  --openssl-root-dir /usr \
  --openssl-lib-dir /usr/lib/aarch64-linux-gnu
```

## Build only (no deploy)

```bash
./target_deploy_script/build_and_deploy_pi4.py --skip-deploy
```

## On the Pi (run only)

```bash
source /opt/ros/humble/setup.bash
source ~/astro/ros2_ws/install/setup.bash
ros2 launch <your_package> <your_launch>.launch.py
```

## Notes

- The script uses separate build output paths (`build_pi4`, `install_pi4`, `log_pi4`) to avoid mixing host-native and Pi-target builds.
- Incremental builds are enabled by default because `build_pi4/` and `install_pi4/` are reused between runs.
- Default package skip includes `dynamixel_sdk_examples` to prevent optional examples from blocking deploy builds; override with `--packages-skip`.
- Docker image is now reused by default if it already exists; use `--rebuild-image` when you change `Dockerfile` or toolchain dependencies.
- For fastest Docker workflow, build a deps image once with `--prepare-deps-image`, then use `--deps-image` in normal runs.
- Runtime assets synced from `src`: `launch`, `config`, `params`, `rviz`, `urdf`, `meshes`.
- Runtime asset sync intentionally does not use `--delete` because it copies only filtered subpaths; this avoids noisy `cannot delete non-empty directory` warnings in remote `src` trees.
- If your package installs all runtime assets correctly, you can use `--no-sync-src-runtime`.
- The script now runs `apt-get update` inside the container before `rosdep install`, which is required when apt indexes are cleaned during image build.
- Dependency checking/install via `rosdep install --from-paths src --ignore-src -r -y` is always enforced before build.
- Default rosdep skip keys are `micro_ros_agent realsense2_camera nav2_bringup`; override with `--rosdep-skip-keys` or `ROSDEP_SKIP_KEYS`.
- Container setup follows CI dependency pattern and includes: `python3-colcon-common-extensions`, `build-essential`, `libyaml-dev`, `git`, plus `python3-rosdep`, `python3-vcstool`, `rsync`.
- `build_and_deploy_pi4.sh` remains as a compatibility wrapper that calls the Python tool.
- rosdep update is now cache-aware by default; use `--force-rosdep-update` when you want a fresh dependency index.

## Troubleshooting

- `exec /bin/sh: exec format error` during Docker build means ARM64 emulation is not set up yet.
- Fix once with:

```bash
docker run --privileged --rm tonistiigi/binfmt --install arm64
```

- Or run the script with `--setup-binfmt` to have it attempt setup automatically.
- `Warning: running 'rosdep update' as root is not recommended.` is expected in Docker container builds and is harmless for this workflow.

## Fast Dev Loop

For quick iterations after code changes:

```bash
./target_deploy_script/build_and_deploy_pi4.py \
  --deps-image \
  --pi ubuntu@astro-pi \
  --packages-select "<changed_pkg_name>"
```

For the fastest loop when dependencies are stable, add `--skip-rosdep`.

If one optional package fails (for example `dynamixel_sdk_examples`), use `--packages-skip` and `--continue-on-error` so unrelated packages still build and deploy.

If `FindOpenSSL` or other CMake `find_package` results look stale after dependency/image updates, prefer `--clean-packages <pkg> --cmake-clean-cache` before doing a full `--clean`.

Advanced flags are still supported; the mode flags above are shortcuts to reduce option overload.

`--full` enables `--rosdep-continue-on-error` so one unavailable apt rosdep key does not stop the build.
