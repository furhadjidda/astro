# Robot deployment troubleshooting

This guide assumes a deployment from the desktop's `ros2/` directory to
`pi@pi.local`, using the default image and remote directory. `pi` and
`pi.local` are examples: substitute your own SSH username and hostname or IP
address. `pi.local` requires working mDNS. Substitute your own `IMAGE` and
`REMOTE_DIR` if they differ. Complete
[Pi setup](PI_SETUP.md), including the device rules, before testing hardware.

**Safety:** secure or lift the wheels before starting or restarting the robot.
The Dynamixel driver enables motor torque. These checks do not command motion.

## 1. Check deployment and service status

On the **desktop**:

```bash
ssh pi@pi.local 'docker info && docker compose version'
ssh pi@pi.local \
  'docker compose -p astro -f ~/astro-robot/docker-compose.yml ps -a'
make logs PI=pi.local PI_USER=pi
```

The `ros` service should be running, not restarting or exited. Logs should show
the expected nodes starting without device or process-death errors. Press
Ctrl+C to stop following logs; this does not stop the robot. A running
container does not prove that every node or device is working.

If `docker: command not found` appears during `make ship`, install Docker
Engine and the Compose plugin **on the Pi** using the
[Ubuntu instructions](https://docs.docker.com/engine/install/ubuntu/) or
[Debian/Raspberry Pi OS instructions](https://docs.docker.com/engine/install/debian/).
After installation, run `sudo usermod -aG docker pi` on the Pi, substituting
your SSH username for `pi`, then log out and reconnect. Docker group membership
grants root-equivalent access. Check that
`ssh pi@pi.local 'docker info && docker compose version'` works before
retrying `make deploy PI=pi.local PI_USER=pi` on the desktop.

If SSH fails, check the address and login with `ssh pi@pi.local`. If Docker
reports permission denied, reconnect after the group change and check `id` on
the Pi. If it cannot connect to the Docker daemon, check
`sudo systemctl status docker` on the Pi. If image loading fails for lack of
space, check `df -h` on the Pi, including the filesystem holding Docker's data;
the built image is large. Do not remove images or data without checking what
else uses them.

### Image transfer appears stuck

The `make ship` pipeline saves the image on the desktop, compresses it with
`gzip`, sends it over SSH, then decompresses and loads it into Docker on the
Pi. It does not compile on the Pi. The pipeline normally shows no progress
until `docker load` finishes; compression, network speed, and Pi disk writes
can all make this slow.

Leave it running and check from **another desktop terminal**:

```bash
docker image ls astro:humble-arm64
ps -eo pid,stat,etime,pcpu,args | grep -E '[d]ocker save|[g]zip|[s]sh pi@'
ssh pi@pi.local 'df -h; ps -eo pid,stat,etime,pcpu,args | grep -E "[g]zip|[d]ocker load"'
```

The image listing is the desktop's uncompressed image size, not the number of
bytes sent. The process list can indicate whether transfer processes still
exist, but does not by itself prove progress. Check disk activity and network
traffic with tools available on the respective machines. On success, expect
`Loaded image: ...`; `make deploy` then copies Compose and starts the service.
Do not start a second deploy while the first is running. If it has genuinely
failed or must be interrupted, inspect the error and retry deployment; an
interrupted load may leave partially downloaded image data on the Pi.

To confirm the Pi and loaded image are ARM64, run on the **Pi**:

```bash
uname -m
docker image inspect astro:humble-arm64 --format '{{.Os}}/{{.Architecture}}'
```

Expect `aarch64` and `linux/arm64`, respectively. Substitute the image tag if
you deployed a different one.

## 2. Inspect ROS inside the container

On the **desktop**, open a shell in the running service:

```bash
make shell PI=pi.local PI_USER=pi
```

In the **container shell**, source ROS explicitly: `docker compose exec`
starts a new process and does not run the image's entrypoint.

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
ros2 pkg list | grep -E 'robot_bringup|dynamixel|sllidar|realsense|depthai|micro_ros_agent'
ros2 node list
ros2 topic list
```

The defaults are domain `10` and `rmw_fastrtps_cpp`. Check logs if an expected
node is missing; a package being installed does not prove its node started.
If another machine cannot see topics, verify matching domain and middleware
configuration, network connectivity, and firewall/multicast settings. Topic
discovery and receipt are separate checks.

Still in the **container shell**:

```bash
ros2 topic info -v /odom
ros2 topic info -v /scan
timeout 15s ros2 topic echo --once --qos-reliability best_effort /odom
timeout 15s ros2 topic echo --once --qos-reliability best_effort /scan
```

Each `echo` should print a live message. A timeout (exit status 124) means no
message arrived in that interval; check publishers, QoS, logs, and hardware.
An `/odom` message alone does not establish that the motors are functioning:
check the Dynamixel node and its logs as well. Use `exit` to leave the shell.

## 3. Check devices

On the **Pi host**, verify the symlinks and their targets:

```bash
ls -l /dev/dynamixel /dev/dynamixel_u2d2 /dev/rplidar /dev/ttyACM0
ls -l /dev/serial/by-id/
```

Missing paths indicate absent devices, wrong or uninstalled udev rules, or
different device names. Inspect the connected devices and compare the actual
IDs with the [udev instructions](PI_SETUP.md#persistent-device-names).
Confirm the same paths exist inside the container. The Dynamixel driver uses
`/dev/dynamixel_u2d2`, the lidar defaults to `/dev/rplidar`, and micro-ROS
defaults to `/dev/ttyACM0`; the latter is not a confirmed persistent path.
Confirm the actual micro-ROS device and configure `microros_dev` if needed.

Check the logs for Dynamixel motor IDs, port and baud errors; for the lidar,
check port, baud rate and scan-mode errors. `/scan` should publish live data.
Persistent names and messages alone do not verify motor IDs or safe motion.

For RealSense, inside the **container shell**:

```bash
rs-enumerate-devices
ros2 node list
ros2 topic list | grep camera
```

The camera should be detected, with its node and expected topics visible.
Enumeration can be affected by another process using the camera. The default
robot launch disables RealSense depth and infrared streams, so don't expect
depth topics from that configuration. Check USB connection, host udev rules,
and launch logs if detection fails.

DepthAI is **not** started by the default robot launch. With no other process
using the OAK, run inside the sourced container shell:

```bash
ros2 launch astro_depthai_test rgb_stereo_smoke.launch.py
```

Check its output and, in a second sourced shell, `ros2 node list` and
`ros2 topic list`. The launch defaults to `camera_model:=OAK-D`; confirm the
actual OAK model and parameters. Stop the test with Ctrl+C.

## 4. Collect useful diagnostics

Capture the service status and recent logs on the **Pi**:

```bash
docker compose -p astro -f ~/astro-robot/docker-compose.yml ps -a
docker compose -p astro -f ~/astro-robot/docker-compose.yml logs --tail=200 ros
```

Include the failing command, its complete error, relevant node/topic names,
and whether the device paths exist on the host and in the container. Review
logs for sensitive information such as addresses and serial numbers before
sharing. A hardware-disabled ARM64 emulation test previously showed the
existing `ros_time` node requiring SIGKILL during shutdown; this has not been
confirmed on the Pi and may be relevant if shutdown or restart hangs.
