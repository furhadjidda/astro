# Repository Tools

Run commands from the repository root unless a command shows a different directory. The build recipes require [just](https://github.com/casey/just#installation); list them at any time with `just`.

## Build and Deployment Tools

| Tool | Purpose | Usage |
|------|---------|-------|
| `just` | Canonical interface for Pico, Zephyr, ROS 2, cross-compilation, and cleanup tasks. | `just`, `just pico pico2_w`, `just zephyr app_sensor_node rak`, `just ros2`, `just ros2-pi4 ubuntu@<pi-ip>` |
| `vcs` | Imports ROS 2 source repositories declared in `ros2/astro.repos`. Used by `just ros2-deps`. | `just ros2-deps` |
| `rosdep` | Installs system dependencies declared by the ROS 2 packages. Used by `just ros2-deps`. | `just ros2-deps` |
| `colcon` | Builds the native ROS 2 workspace. Wrapped by `just ros2`. | `just ros2` |
| `west` | Initializes, builds, and flashes Zephyr applications. Wrapped by the Zephyr `just` recipes. | `just zephyr-init`; `just zephyr <app> <rak|feather>` |
| `ros2/deploy/build_and_deploy_pi4.py` | Builds the ROS 2 workspace for ARM64 in Docker/QEMU and deploys it to a Pi with `rsync`. | `just ros2-pi4 ubuntu@<pi-ip> --first-run`; build only: `just ros2-pi4` |

Refer to [Build, Flash, and Use Astro](build-flash-use.md) for prerequisites and the complete build and deployment sequence.

## Device and Firmware Utilities

| Tool | Purpose | Usage |
|------|---------|-------|
| `tools/pico_reboot_and_copy/pico_reboot_and_copy.py` | Reboots a running Pico through its micro-ROS `reboot_to_bootloader` service, detects its UF2 mass-storage volume, and offers a GUI to copy a chosen firmware image. | Source ROS 2 and the workspace, then run `python3 tools/pico_reboot_and_copy/pico_reboot_and_copy.py firmware/pico/build_pico2_w` |
| `tools/serial_monitor/astromon.py` | Streams a serial device, colorizes `err`, `dbg`, and `inf` log lines, writes a timestamped log, and reconnects after disconnects. | `python3 tools/serial_monitor/astromon.py /dev/rak_wireless_esp32s3 115200` |
| `tools/monitor_devices/monitor_devices.py` | Watches udev add events, prints device metadata and created `/dev` symlinks, and sends desktop notifications. | `python3 tools/monitor_devices/monitor_devices.py` |
| `tools/monitor_devices/install.py` | Installs, starts, removes, or disables the device monitor as a systemd user service. | Install: `python3 tools/monitor_devices/install.py`; remove: `python3 tools/monitor_devices/install.py --remove` |

### Utility Dependencies

```bash
sudo apt install python3-tk libnotify-bin
python3 -m pip install pyserial pyudev
```

The Pico uploader also needs a sourced ROS 2 environment and `rclpy`, normally provided by the ROS installation. It requires a running firmware image that exposes `reboot_to_bootloader`; use BOOTSEL and manual UF2 copy when that service is unavailable.

## Source Transfer Utilities

| Tool | Purpose | Usage |
|------|---------|-------|
| `tools/transfer_to_target/transfer_to_target_ui.py` | GUI SCP client for uploading a selected directory to an Astro target. The destination defaults to `/home/astro/ros2_ws`; settings are stored beside the script. | `cd tools/transfer_to_target && python3 transfer_to_target_ui.py` |
| `tools/transfer_to_target/transfer_to_target.py` | Legacy command-line SCP client intended to upload a fixed ROS 2 source set. | Do not use for new deployments; use `just ros2-pi4 ubuntu@<pi-ip>` instead. |

The transfer GUI requires `paramiko`, `scp`, and `PyQt5`:

```bash
python3 -m pip install paramiko scp PyQt5
```

Both transfer clients currently use password authentication and automatically accept unknown host keys. Use only on a trusted network. The CLI references a `ros2/utils` directory that is not present in this repository, so it is retained as a legacy utility rather than a supported deployment path.

## Documentation Tooling

| Tool | Purpose | Usage |
|------|---------|-------|
| Jekyll / GitHub Pages | Renders the site under `docs/`. | `cd docs && bundle install && bundle exec jekyll serve` |
| `docs/diagrams/encode_puml.py` | Encodes PlantUML sources for embedded diagram URLs. | See [Diagram Authoring Guide](DIAGRAMS.md) |

The documentation site is configured by `docs/_config.yml` and its Ruby dependencies are declared in `docs/Gemfile`.