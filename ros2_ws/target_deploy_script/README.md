# Build ROS 2 for Raspberry Pi 4 on Host

Cross-compiles the Astro ROS 2 workspace for Raspberry Pi 4 (ARM64) inside a Docker container on your development machine, then deploys the result to the Pi over `rsync`.

Script: `build_and_deploy_pi4.py`

---

## What it does

1. Builds an ARM64 Docker image containing ROS tools + depthai binaries.
2. Runs `colcon build` inside that container (ARM64 via QEMU emulation).
3. Produces `install_pi4/` — the Pi-ready install tree.
4. Rsyncs `install_pi4/` and runtime assets (`launch`, `config`, `params`, …) to the Pi.

---

## Prerequisites

| Requirement | Notes |
|---|---|
| Docker with `buildx` | Comes with Docker Desktop; on Linux `apt install docker-buildx` |
| QEMU binfmt for ARM64 | One-time setup — see below |
| SSH access to Pi | Key-based recommended |
| ROS Humble on Pi | Must match the distro used to build |

### One-time ARM64 emulation setup

```bash
docker run --privileged --rm tonistiigi/binfmt --install arm64
```

Verify it works:

```bash
docker run --rm --platform linux/arm64 alpine uname -m   # should print aarch64
```

---

## Common commands

All commands are run from the `ros2_ws/` directory.

### First-time build + deploy

```bash
./target_deploy_script/build_and_deploy_pi4.py --pi ubuntu@<pi-ip> --first-run
```

This builds the Docker image (if it doesn't exist), installs missing apt/rosdep dependencies, compiles the workspace, and syncs to the Pi.

### Everyday quick build + deploy (after code changes)

```bash
./target_deploy_script/build_and_deploy_pi4.py --pi ubuntu@<pi-ip>
```

Docker image is reused automatically. Only changed packages are recompiled.
Dependency installation is skipped in this quick path for speed.

### Build only (no deploy)

```bash
./target_deploy_script/build_and_deploy_pi4.py --skip-deploy
```

### Build a specific package only

```bash
./target_deploy_script/build_and_deploy_pi4.py \
  --packages-select astro_sensor_publisher \
  --skip-deploy
```

### Full clean rebuild

Use this after Dockerfile changes or when you suspect stale build state:

```bash
./target_deploy_script/build_and_deploy_pi4.py \
  --clean --rebuild-image --install-deps --skip-deploy
```

---

## Start from scratch (Docker cleanup)

### Prune builder image + containers + dangling cache

```bash
./target_deploy_script/build_and_deploy_pi4.py --prune-docker
```

This removes:
- The `astro-ros2-pi4-builder:humble` image
- Stopped containers
- Dangling build cache

Then rebuild from scratch:

```bash
./target_deploy_script/build_and_deploy_pi4.py --clean --skip-deploy
```

### Nuclear option — remove ALL unused Docker images

```bash
docker image prune -af
docker builder prune -af
```

---

## All options

```
--pi <user@host>          SSH target for the Pi  [env: PI_TARGET]
--pi-dir <path>           Remote workspace path  [env: PI_DIR]  (default: ~/astro/ros2_ws)
--ros-distro <distro>     ROS distro  [env: ROS_DISTRO]         (default: humble)
--image-name <name>       Override Docker image name
--rebuild-image           Force rebuild Docker image
--install-deps            Run apt/rosdep dependency installation before build (slower)
--first-run               Alias for --install-deps (easier first-time command)
--skip-deploy             Build only; skip rsync to Pi
--clean                   Delete build_pi4 / install_pi4 / log_pi4 before building
--continue-on-error       Keep building even if a package fails
--packages-select <pkgs>  Build only these packages
--packages-up-to <pkgs>   Build up to and including these packages
--packages-skip <pkgs>    Skip these packages  [env: PACKAGES_SKIP]
--build-base              Build directory name  (default: build_pi4)
--install-base            Install directory name  (default: install_pi4)
--log-base                Log directory name  (default: log_pi4)
--prune-docker            Remove builder image, prune containers and build cache, then exit
```

---

## On the Pi (run)

```bash
source /opt/ros/humble/setup.bash
source ~/astro/ros2_ws/install/setup.bash
ros2 launch robot_bringup bringup.launch.py
```

---

## Notes

- Build output uses separate paths (`build_pi4`, `install_pi4`, `log_pi4`) so host-native and Pi-target builds never mix.
- depthai ROS packages (`depthai-ros-v3`) are installed from apt inside the Docker image — no source build needed.
- `vision_msgs` is also installed as an apt dependency of `depthai-ros-v3`, removing the need to clone it via `astro.repos`.
- Dependency installation is opt-in via `--install-deps` (recommended for first run, dependency updates, or after fresh Docker prune).
- Default skip: `dynamixel_sdk_examples`; override with `--packages-skip`.

---

## Troubleshooting

**`exec /bin/sh: exec format error`**
ARM64 emulation is not set up. Run:
```bash
docker run --privileged --rm tonistiigi/binfmt --install arm64
```

**`docker: Error response from daemon: … image not found`**
The builder image doesn't exist yet. It will be built automatically on the first run, or force it with `--rebuild-image`.

**Stale cmake cache after changing dependencies**
Run with `--clean` to delete `build_pi4` and `install_pi4` before the next build.
