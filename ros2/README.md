# ROS 2 development and Pi 4 deployment

Develop on the x86-64 desktop and build a `linux/arm64` image with Docker
Buildx and QEMU. The Pi runs the resulting image; it does not compile the
workspace. The default deploy transfers the complete compressed image over SSH.

Examples use `pi.local` as a placeholder for the Pi's hostname and `pi` as
its SSH username. Replace both with your own hostname or IP address and
username; `pi.local` requires working mDNS. Run desktop commands from this
`ros2/` directory unless stated otherwise.

Commands below identify three environments:

- **Desktop host:** your x86-64 development machine, in the `ros2/` directory.
- **Pi host:** the Raspberry Pi operating system, reached with
  `ssh pi@pi.local` from the desktop; not the container.
- **Pi container:** the running ROS container, entered with `make shell` from
  the desktop. Run `exit` there to return to the desktop.

An SSH command is typed on the desktop even when the quoted commands execute
on the Pi. Commands in the Pi host tables are typed after connecting over SSH;
container commands are typed after entering the container.

**Safety:** secure or lift the wheels before starting or restarting the robot.
The Dynamixel driver enables motor torque at startup. The container runs
privileged with host networking and access to host devices; Docker group and
container access are effectively root access.

## One-time setup

1. On the desktop, install Docker Engine, Buildx, and `make`. Verify that the
   desktop user can run Docker without interactive sudo.
2. Set up the 64-bit Pi, SSH, Docker Engine, Compose plugin, and device rules
   following [Pi setup](docs/PI_SETUP.md). Confirm the actual USB identifiers,
   micro-ROS device path, and required Pi interfaces. The rules must provide
   `/dev/dynamixel_u2d2` (also aliased as `/dev/dynamixel`) and `/dev/rplidar`.
3. **Desktop host:** verify that SSH and Docker work without interactive sudo.
   The quoted commands execute on the Pi:

   ```bash
   ssh pi@pi.local 'uname -m && docker info && docker compose version'
   ```

   The architecture should be `aarch64`. If Docker is missing on the Pi,
   install it before adding the Pi user to the `docker` group. Reconnect after
   changing group membership.
4. **Desktop host:** set up ARM64 emulation once:

   ```bash
   make setup-buildx
   ```

   This installs ARM64 binfmt on the desktop host and requires Docker
   privileged access; it changes host-wide emulation configuration.

## First build and deployment

**Desktop host** (`ros2/`):

```bash
make deploy PI=pi.local PI_USER=pi
```

This builds the image, saves and compresses it, streams it to the Pi over SSH,
loads it into Docker, copies Compose to `~/astro-robot/`, and starts the
service. The transfer can take a while and normally shows no progress until
loading finishes. No code is compiled on the Pi.

The service is named `ros` in Compose project `astro`, so the container is
normally named `astro-ros-1`. It runs
`ros2 launch robot_bringup robot_base.launch.py`, with `ROS_DOMAIN_ID=10` and
`RMW_IMPLEMENTATION=rmw_fastrtps_cpp` by default. DepthAI is not started by
this launch; see the [DepthAI smoke test](docs/PI_SETUP.md#pi-smoke-tests).

The URDF defaults to `waffle`. To deploy a burger instead, run
`make deploy PI=pi.local PI_USER=pi ROBOT_MODEL=burger`. This selects the URDF
at runtime; it does not change odometry settings. For direct ROS launches,
including Nav2 on another machine, set `ASTRO_ROBOT_MODEL=burger` in that
machine's environment. See [navigation configuration](src/astro_navigation/README.md)
before running a burger autonomously.

## Change, deploy, and verify

1. **Desktop host:** edit the ROS packages, including launch/configuration files
   as needed.
2. Secure the robot, then on the **desktop host** run
   `make deploy PI=pi.local PI_USER=pi` again.
   The build reuses cached external layers and `ccache` where possible, but
   SSH transfer still sends the **whole compressed image**. Deployment
   overwrites the Pi's Compose file with the desktop copy.
3. On the **desktop host**, follow logs with
   `make logs PI=pi.local PI_USER=pi` and check for errors or nodes that
   died. Ctrl+C stops following logs, not the robot.
4. Use the container shell and checks below to verify nodes, topics, and
   hardware. A running container does not guarantee that each node or device
   is working. Changes made only inside the container are not a persistent
   development workflow; edit the desktop workspace and redeploy instead.

GitHub Actions builds and checks the ARM64 image on pushes to `main` and pull
requests targeting `main`, and publishes eligible builds to GHCR as described
below. It does not deploy or test Pi hardware. External Git sources follow
branches and may move. Docker can reuse a cached fetch even after a branch
advances; see [Pi setup](docs/PI_SETUP.md#build-and-deploy) for how to refresh
it deliberately.

## Faster development with package overlays

This optional SSH workflow builds selected packages on the **desktop** for
ARM64, transfers only their merged install overlay, and recreates the Pi
service. It does not compile on the Pi. It is for development: make a full
image for releases or when dependencies, interfaces, or build configuration
change. **Secure or lift the wheels before every command that starts or
restarts the service.**

First, from the **desktop host** in `ros2/`, create a baseline:

```bash
make overlay-init PI=pi.local PI_USER=pi
```

This performs a full build and SSH deployment, then snapshots the baseline
sources, install files, and builder package metadata under `ros2/.overlay/`. It may be
slow and requires disk space. It overwrites the Pi Compose file just like
`make deploy`. Run it again to establish a new baseline after a full image,
package manifest, dependency, Dockerfile, entrypoint, or Compose change.

After editing a package on the desktop, build and deploy it:

```bash
make overlay-deploy PI=pi.local PI_USER=pi PACKAGE=robot_bringup
```

Use the ROS **package name**, not necessarily its directory name. Each update
rebuilds that package, its declared workspace dependents, and packages
previously selected for this baseline. Dependencies must be declared correctly
in package manifests; undeclared or external consumers and ABI compatibility
are not automatically verified. Rebuild the full image for interface or
dependency changes. The workflow checks the Pi image's platform, layers and
configuration, local package manifests, baseline install files, and builder
package versions, but these checks cannot prove hardware or ABI compatibility.

The overlay remains active after a container or Pi restart. To return to the
baseline image, from the **desktop host**:

```bash
make overlay-reset PI=pi.local PI_USER=pi
```

Reset recreates the service without the overlay and clears the selected package
list. Old overlay releases on the Pi and baseline snapshots on the desktop are
retained for inspection and consume disk space. An ordinary `make deploy`
also recreates the service without the overlay; run `make overlay-init` before
using overlay updates again. Use the same `IMAGE`, `REMOTE_DIR`, ROS variables,
and build parallelism settings as the baseline. Do not edit the Pi Compose file
independently while using this workflow. The baseline builder relies on BuildKit
cache; if it is evicted, rebuilding it can be slow or fail the fingerprint
check, requiring a new baseline.

Use the same `ROBOT_MODEL` on `overlay-init`, `overlay-deploy`, and
`overlay-reset`; for a burger baseline, add `ROBOT_MODEL=burger` to each.

To inspect an active overlay, enter the container with `make shell` and source
the overlay **after** the baseline (an exec shell does not run the entrypoint):

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
source /opt/astro-overlay/local_setup.bash
ros2 pkg prefix robot_bringup
```

For an overlaid package, the prefix should be `/opt/astro-overlay`. Check logs
and hardware as described below. Overlay deployment does not push an image to
GHCR and is not part of CI. If an update fails after restarting the service,
inspect its status and logs; retry or run `make overlay-reset` rather than
assuming the old overlay is still active.

## Command reference

### Desktop host commands

Run from the desktop's `ros2/` directory:

| Purpose | Command |
| --- | --- |
| Set up ARM64 emulation once | `make setup-buildx` |
| Build and load the ARM64 image locally | `make build` |
| Transfer an already-built image only | `make ship PI=pi.local PI_USER=pi` |
| Build, transfer, and start/update the robot | `make deploy PI=pi.local PI_USER=pi` |
| Create a full-image development baseline | `make overlay-init PI=pi.local PI_USER=pi` |
| Build and deploy a package overlay | `make overlay-deploy PI=pi.local PI_USER=pi PACKAGE=robot_bringup` |
| Return to the baseline image | `make overlay-reset PI=pi.local PI_USER=pi` |
| Follow service logs | `make logs PI=pi.local PI_USER=pi` |
| Open a shell in the running container | `make shell PI=pi.local PI_USER=pi` |
| Connect to the Pi host | `ssh pi@pi.local` |

`PI_USER` defaults to `astro`. To use `PI=user@host`, explicitly set
`PI_USER=`. `IMAGE` defaults to `astro:humble-arm64` and `REMOTE_DIR` to
`astro-robot`; pass the same values to later commands if you override them.
`BUILD_JOBS` and `PARALLEL_WORKERS` default to 2. See
[Pi setup](docs/PI_SETUP.md#build-and-deploy) for other overrides.

### Pi host commands

First connect from the **desktop host**:

```bash
ssh pi@pi.local
```

Then run the following in the **Pi host** SSH session, not in the container:

| Purpose | Command |
| --- | --- |
| Show status, including stopped services | `docker compose -p astro -f ~/astro-robot/docker-compose.yml ps -a` |
| Follow logs | `docker compose -p astro -f ~/astro-robot/docker-compose.yml logs -f ros` |
| Stop all nodes in the service | `docker compose -p astro -f ~/astro-robot/docker-compose.yml stop ros` |
| Start the previously created service | `docker compose -p astro -f ~/astro-robot/docker-compose.yml start ros` |
| Verify host device names | `ls -l /dev/dynamixel /dev/dynamixel_u2d2 /dev/rplidar /dev/ttyACM0` |

The service normally restarts after a Pi reboot if Docker starts on boot;
`restart: unless-stopped` means a manually stopped service remains stopped.
Device readiness and individual node health still need checking after boot.
Run `exit` on the Pi host to return to the desktop.

## Shell and diagnose

**Desktop host** (`ros2/`): enter the running container:

```bash
make shell PI=pi.local PI_USER=pi
```

**Pi container:** source ROS in the new shell:

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
```

Run the following inside the **Pi container**:

| Purpose | Command inside the Pi container |
| --- | --- |
| List installed packages | `ros2 pkg list` |
| List running nodes | `ros2 node list` |
| List topics | `ros2 topic list` |
| Inspect lidar publishers and QoS | `ros2 topic info -v /scan` |
| Check for a lidar message | `timeout 15s ros2 topic echo --once --qos-reliability best_effort /scan` |
| Check for an odometry message | `timeout 15s ros2 topic echo --once --qos-reliability best_effort /odom` |
| Check device names visible to the container | `ls -l /dev/dynamixel /dev/dynamixel_u2d2 /dev/rplidar` |
| Enumerate RealSense devices | `rs-enumerate-devices` |
| Leave the container shell | `exit` |

A timeout means no message arrived within 15 seconds. Check publishers, QoS,
logs, and hardware; an `/odom` message alone does not prove motor operation.
RealSense enumeration may be affected if another process owns the camera.
See [troubleshooting](docs/TROUBLESHOOTING.md) for transfer, Docker, ROS, and
device diagnostics.

## Optional: transfer through a registry

A registry lets the Pi reuse image layers it already has. The first upload and
download are large; a small source change may still change the whole merged ROS
install layer. Pulling and starting a registry image on the Pi is a **manual
alternative** to `make deploy`.

### CI-published images

After building and checking the image, GitHub Actions publishes to
`ghcr.io/<repository-owner>/astro` for pushes to `main` and eligible PRs
targeting `main` from the same repository:

| Event | Tag |
| --- | --- |
| Push to `main` | `main-<full-commit-sha>` |
| Same-repository PR | `pr-<number>-<full-commit-sha>` |

For PRs, the SHA identifies the merge commit checked out by Actions, not
necessarily the PR head commit. The workflow summary shows the exact image
reference. Fork and Dependabot PRs are built and checked but not published
because their tokens normally cannot write packages. CI never deploys to the
Pi. A rerun can replace a tag, especially since external sources follow
branches; use the image digest for an immutable reference.

Publishing uses `GITHUB_TOKEN` with package write permission; no personal token
is needed in CI. If the GHCR package was created previously, connect it to the
repository and grant the repository's Actions workflow write access in the
package settings. See [GitHub's package access instructions](https://docs.github.com/en/packages/learn-github-packages/configuring-a-packages-access-control-and-visibility).
New packages are private by default. Only trusted users should have access to
create same-repository branches and run publish-capable workflows.

To try a published tag, follow the Pi login and pull steps below, substituting
the exact reference from the workflow summary for `IMAGE`. Inspect the push
log or registry for its digest and use `ghcr.io/<owner>/astro@sha256:<digest>`
to select an immutable image. The Pi needs the matching Compose file: the
commands below use the copy from an earlier SSH deployment, so update it
separately if the PR changes Compose.

### Manual push and Pi pull

For a manual GitHub Container Registry (GHCR) workflow:

1. **Desktop host:** create a GitHub **classic** personal access token with
   `write:packages` and log in. Enter the token at the prompt, not in the
   command or chat. Replace `your-github-username` with your GitHub username
   in this and the following commands:

   ```bash
   read -rsp 'GitHub token: ' TOKEN; echo
   printf '%s' "$TOKEN" | docker login ghcr.io -u your-github-username --password-stdin
   unset TOKEN
   ```

   Docker may store credentials on the machine. See
   [GitHub's GHCR instructions](https://docs.github.com/en/packages/working-with-a-github-packages-registry/working-with-the-container-registry).
2. **Desktop host** (`ros2/`): build and push a uniquely tagged image. Change the
   namespace and tag as needed:

   ```bash
   IMAGE=ghcr.io/your-github-username/astro:humble-arm64-20261121-1
   make build IMAGE="$IMAGE" && docker push "$IMAGE"
   ```

3. **Pi host:** connect using `ssh pi@pi.local` from the desktop. New GHCR
   packages are private by default, so log in on the Pi with a separate classic
   token with `read:packages`. Enter the token at the prompt:

   ```bash
   read -rsp 'GitHub token: ' TOKEN; echo
   printf '%s' "$TOKEN" | docker login ghcr.io -u your-github-username --password-stdin
   unset TOKEN
   ```

   The Pi needs network access to the registry and a copy of the Compose file,
   as provided by the first SSH deployment above.
4. **Pi host:** secure the robot and pull and start that exact tag:

   ```bash
   cd ~/astro-robot
   export IMAGE=ghcr.io/your-github-username/astro:humble-arm64-20261121-1
   docker pull "$IMAGE" && docker compose -p astro up -d
   docker compose -p astro logs -f ros
   ```

For the next change, use a CI-published tag or build and push a new tag, then
pull and start it on the Pi. Keep old tags as needed for rollback.
