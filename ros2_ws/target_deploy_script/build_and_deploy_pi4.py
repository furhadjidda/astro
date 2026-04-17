#!/usr/bin/env python3
"""Cross-compile the Astro ROS 2 workspace for Raspberry Pi 4 (ARM64) and optionally deploy it.

Typical usage
─────────────
# Build and deploy to Pi (first time or after changing code):
    ./build_and_deploy_pi4.py --pi ubuntu@192.168.1.120 --first-run

# Build only, no deploy:
  ./build_and_deploy_pi4.py --skip-deploy

# Force-rebuild the Docker image (e.g. after Dockerfile changes):
  ./build_and_deploy_pi4.py --rebuild-image --skip-deploy

# Full clean rebuild:
  ./build_and_deploy_pi4.py --clean --rebuild-image --skip-deploy

# Wipe Docker builder image + prune containers/cache to start from scratch:
  ./build_and_deploy_pi4.py --prune-docker

# Build only specific packages:
  ./build_and_deploy_pi4.py --packages-select astro_sensor_publisher --skip-deploy

# Fast daily build + deploy (skip rosdep dependency install):
    ./build_and_deploy_pi4.py --pi ubuntu@192.168.1.120
"""

import argparse
import os
import re
import shlex
import shutil
import subprocess
import sys
from pathlib import Path


# ── Helpers ────────────────────────────────────────────────────────────────────


class _Color:
    RED = "\033[31m"
    GREEN = "\033[32m"
    YELLOW = "\033[33m"
    RESET = "\033[0m"


def print_error(msg: str) -> None:
    print(f"{_Color.RED}{msg}{_Color.RESET}", file=sys.stderr)


def print_warn(msg: str) -> None:
    print(f"{_Color.YELLOW}{msg}{_Color.RESET}")


def print_success(msg: str) -> None:
    print(f"{_Color.GREEN}{msg}{_Color.RESET}")


def run(cmd: list[str], *, check: bool = True) -> int:
    print("+", " ".join(shlex.quote(x) for x in cmd))
    completed = subprocess.run(cmd)
    if check and completed.returncode != 0:
        raise SystemExit(completed.returncode)
    return completed.returncode


def ensure_docker() -> None:
    if (
        subprocess.run(["bash", "-lc", "command -v docker >/dev/null 2>&1"]).returncode
        != 0
    ):
        print_error("docker is required but was not found")
        raise SystemExit(1)


def arm64_emulation_ready() -> bool:
    return (
        subprocess.run(
            [
                "docker",
                "run",
                "--rm",
                "--platform",
                "linux/arm64",
                "alpine:3.20",
                "uname",
                "-m",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        ).returncode
        == 0
    )


def docker_image_exists(image_name: str) -> bool:
    return (
        subprocess.run(
            ["docker", "image", "inspect", image_name],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        ).returncode
        == 0
    )


def find_workspace_root(start: Path) -> Path:
    for candidate in [start, *start.parents]:
        if (candidate / "src").is_dir() and (candidate / "astro.repos").is_file():
            return candidate
    raise SystemExit(
        f"{_Color.RED}Could not find workspace root "
        f"(looked for a directory containing src/ and astro.repos).{_Color.RESET}"
    )


# ── CLI ────────────────────────────────────────────────────────────────────────


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Cross-compile ROS 2 workspace for Raspberry Pi 4 (ARM64) "
        "and optionally deploy.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )

    # Deployment target
    p.add_argument(
        "--pi",
        dest="pi_target",
        default=os.environ.get("PI_TARGET", ""),
        help="SSH target for your Pi, e.g. ubuntu@192.168.1.120  [env: PI_TARGET]",
    )
    p.add_argument(
        "--pi-dir",
        default=os.environ.get("PI_DIR", "~/astro/ros2_ws"),
        help="Remote workspace path on the Pi  [env: PI_DIR]",
    )

    # ROS / Docker
    p.add_argument(
        "--ros-distro",
        default=os.environ.get("ROS_DISTRO", "humble"),
        help="ROS distro to build for  [env: ROS_DISTRO]",
    )
    p.add_argument(
        "--image-name",
        default=os.environ.get("IMAGE_NAME", ""),
        help="Override the Docker image name",
    )
    p.add_argument(
        "--rebuild-image",
        action="store_true",
        help="Force rebuild the Docker image even if it already exists",
    )
    p.add_argument(
        "--install-deps",
        action="store_true",
        help="Run apt/rosdep dependency installation inside the container before build (slower, use for first run or after dependency changes)",
    )
    p.add_argument(
        "--first-run",
        action="store_true",
        help="Alias for --install-deps (recommended for first setup on a machine)",
    )

    # Build control
    p.add_argument(
        "--skip-deploy", action="store_true", help="Build only; do not rsync to Pi"
    )
    p.add_argument(
        "--clean",
        action="store_true",
        help="Delete build/install/log directories before building",
    )
    p.add_argument(
        "--continue-on-error",
        action="store_true",
        help="Let colcon keep building if one package fails",
    )
    p.add_argument(
        "--packages-select",
        default="",
        help="Build only these packages (space-separated)",
    )
    p.add_argument(
        "--packages-up-to",
        default="",
        help="Build up to and including these packages (space-separated)",
    )
    p.add_argument(
        "--packages-skip",
        default=os.environ.get("PACKAGES_SKIP", "dynamixel_sdk_examples"),
        help="Skip these packages (space-separated)  [env: PACKAGES_SKIP]",
    )

    # Directory names (rarely need changing)
    p.add_argument("--build-base", default=os.environ.get("BUILD_BASE", "build_pi4"))
    p.add_argument(
        "--install-base", default=os.environ.get("INSTALL_BASE", "install_pi4")
    )
    p.add_argument("--log-base", default=os.environ.get("LOG_BASE", "log_pi4"))

    # Maintenance
    p.add_argument(
        "--prune-docker",
        action="store_true",
        help="Remove the builder Docker image, prune stopped containers "
        "and dangling build cache, then exit.  Use this to start "
        "completely fresh.",
    )

    return p.parse_args()


# ── Main ───────────────────────────────────────────────────────────────────────


def main() -> int:
    args = parse_args()

    # Convenience alias.
    if args.first_run:
        args.install_deps = True

    script_dir = Path(__file__).resolve().parent
    ws_root = find_workspace_root(script_dir)
    base_paths = [p for p in ("src", "utils") if (ws_root / p).is_dir()]
    if not base_paths:
        print_error(
            "Workspace does not contain any base paths to build (expected src and/or utils)."
        )
        return 1
    dockerfile = next(
        (
            script_dir / n
            for n in ("Dockerfile", "dockerfile")
            if (script_dir / n).is_file()
        ),
        None,
    )
    if dockerfile is None:
        print_error(f"Dockerfile not found in {script_dir}")
        return 1

    image_name = args.image_name or f"astro-ros2-pi4-builder:{args.ros_distro}"

    ensure_docker()

    # ── Prune / start-from-scratch ─────────────────────────────────────────────
    if args.prune_docker:
        print_success("Removing builder image (if it exists) …")
        subprocess.run(["docker", "rmi", "-f", image_name], check=False)
        print_success("Pruning stopped containers …")
        run(["docker", "container", "prune", "-f"])
        print_success("Pruning dangling images …")
        run(["docker", "image", "prune", "-f"])
        print_success("Pruning dangling build cache …")
        run(["docker", "builder", "prune", "-f"])
        print_success(
            "Done. Run the script again (without --prune-docker) to rebuild from scratch."
        )
        print_warn(
            "Tip: to also remove ALL unused Docker images (not just dangling ones) run:\n"
            "  docker image prune -af"
        )
        return 0

    # ── ARM64 emulation check ──────────────────────────────────────────────────
    if not arm64_emulation_ready():
        print_error("ARM64 container emulation is not configured.")
        print_error("Run this once, then retry:")
        print_error("  docker run --privileged --rm tonistiigi/binfmt --install arm64")
        return 1

    if not args.skip_deploy and not args.pi_target:
        print_error("--pi is required unless --skip-deploy is used.")
        return 1

    # ── [1/4] Build Docker image ───────────────────────────────────────────────
    image_exists = docker_image_exists(image_name)
    if args.rebuild_image or not image_exists:
        print_success(f"[1/4] Building ARM64 Docker image: {image_name}")
        run(
            [
                "docker",
                "buildx",
                "build",
                "--platform",
                "linux/arm64",
                "--load",
                "--build-arg",
                f"ROS_DISTRO={args.ros_distro}",
                "-t",
                image_name,
                "-f",
                str(dockerfile),
                str(ws_root),
            ]
        )
    else:
        print_success(
            f"[1/4] Reusing existing image {image_name}  "
            "(pass --rebuild-image to rebuild)"
        )

    # ── [2/4] Optional clean ───────────────────────────────────────────────────
    if args.clean:
        print_success("[2/4] Removing build / install / log directories …")
        for d in (args.build_base, args.install_base, args.log_base):
            target = ws_root / d
            if target.exists():
                shutil.rmtree(target)
                print(f"  removed {target}")
    else:
        print_success("[2/4] Skipping clean (pass --clean for a full rebuild)")

    # ── [3/4] Build inside ARM64 container ────────────────────────────────────
    print_success("[3/4] Building workspace for ARM64 …")

    # Keys to skip during rosdep install:
    #   micro_ros_agent   – built separately for Zephyr, not in rosdep index
    #   librealsense2     – installed from Intel's apt repo in the Dockerfile;
    #                       rosdep maps this to ros-humble-librealsense2 which
    #                       does not exist as an apt package
    rosdep_skip_keys = "micro_ros_agent librealsense2"

    colcon_cmd: list[str] = [
        "colcon",
        "--log-base",
        args.log_base,
        "build",
        "--base-paths",
        *base_paths,
        "--merge-install",
        "--build-base",
        args.build_base,
        "--install-base",
        args.install_base,
        "--continue-on-error",
    ]
    if args.packages_up_to.strip():
        colcon_cmd += ["--packages-up-to", *shlex.split(args.packages_up_to)]
    if args.packages_select.strip():
        colcon_cmd += ["--packages-select", *shlex.split(args.packages_select)]
    if args.packages_skip.strip():
        colcon_cmd += ["--packages-skip", *shlex.split(args.packages_skip)]

    colcon_cmd += [
        "--cmake-args",
        "--no-warn-unused-cli",
        "-DCMAKE_BUILD_TYPE=Release",
        # Python paths specified explicitly because cmake under qemu ARM64
        # emulation sometimes resolves the wrong python3 via pkg-config.
        "-DPYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.10.so",
        "-DPYTHON_LIBRARIES=/usr/lib/aarch64-linux-gnu/libpython3.10.so",
        "-DPYTHON_INCLUDE_DIR=/usr/include/python3.10",
        "-DPYTHON_INCLUDE_DIRS=/usr/include/python3.10",
        "-DTINYXML2_LIBRARY=/usr/lib/aarch64-linux-gnu/libtinyxml2.so",
        "-DTINYXML2_INCLUDE_DIR=/usr/include",
    ]

    container_steps: list[str] = [
        "set -e",
        "export CMAKE_BUILD_PARALLEL_LEVEL=${CMAKE_BUILD_PARALLEL_LEVEL:-$(nproc)}",
        f"source /opt/ros/{shlex.quote(args.ros_distro)}/setup.bash",
        # Source local install overlay so AMENT_PREFIX_PATH includes any
        # previously-built packages; ament_cmake reads this env var via
        # ament_index_get_prefix_path() to locate cmake package configs.
        f'if [ -n "${{CI:-}}" ]; then'
        f" echo 'CI detected: skipping existing /ws/{args.install_base} overlay';"
        f" elif [ -f /ws/{args.install_base}/setup.bash ];"
        f" then source /ws/{args.install_base}/setup.bash; fi",
        f"export LD_LIBRARY_PATH=/ws/{args.install_base}/lib"
        f":/opt/ros/{shlex.quote(args.ros_distro)}/lib:${{LD_LIBRARY_PATH:-}}",
    ]

    if args.install_deps:
        rosdep_from_paths = " ".join(shlex.quote(p) for p in base_paths)
        container_steps.extend(
            [
                # Refresh apt lists (cleared during image build) then run rosdep.
                "apt-get update -qq",
                # Ensure core ROS interface/runtime libraries exist in the live container.
                (
                    "apt-get install -y --no-install-recommends "
                    f"ros-{shlex.quote(args.ros_distro)}-ros-base "
                    f"ros-{shlex.quote(args.ros_distro)}-rcutils "
                    f"ros-{shlex.quote(args.ros_distro)}-builtin-interfaces "
                    f"ros-{shlex.quote(args.ros_distro)}-rosidl-default-generators"
                ),
                (
                    f"test -f /opt/ros/{shlex.quote(args.ros_distro)}/lib/librcutils.so"
                    f" && test -f /opt/ros/{shlex.quote(args.ros_distro)}/lib/"
                    "libbuiltin_interfaces__rosidl_generator_c.so"
                ),
                # Initialize rosdep and install any remaining deps from source packages.
                "if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ];"
                " then rosdep init; fi",
                "if [ ! -d /root/.ros/rosdep/sources.cache ]; then rosdep update; fi",
                (
                    f"rosdep install --from-paths {rosdep_from_paths} --ignore-src -r -y"
                    f" --rosdistro {shlex.quote(args.ros_distro)}"
                    f" --skip-keys {shlex.quote(rosdep_skip_keys)}"
                    f" || true"
                ),
            ]
        )
    else:
        print_success("[3/4] Skipping dependency installation (fast mode).")
        print_warn(
            "Use --install-deps on first run or after adding/changing package dependencies."
        )

    container_steps.append(shlex.join(colcon_cmd))

    run(
        [
            "docker",
            "run",
            "--rm",
            "--platform",
            "linux/arm64",
            "-v",
            f"{ws_root}:/ws",
            "-w",
            "/ws",
            image_name,
            "bash",
            "-lc",
            " && ".join(container_steps),
        ]
    )

    # ── [4/4] Deploy ──────────────────────────────────────────────────────────
    if args.skip_deploy:
        print_success("[4/4] Deploy skipped.")
        print_success(f"Build output: {ws_root / args.install_base}")
        return 0

    # Guard against user/path mismatch in --pi and --pi-dir.
    if "@" in args.pi_target and args.pi_dir.startswith("/home/"):
        remote_user = args.pi_target.split("@", 1)[0]
        m = re.match(r"^/home/([^/]+)", args.pi_dir)
        if m and m.group(1) != remote_user:
            print_error(
                f"Path mismatch: --pi user is '{remote_user}' "
                f"but --pi-dir is under '/home/{m.group(1)}'.\n"
                f"Did you mean --pi-dir /home/{remote_user}/ros2_ws ?"
            )
            return 1

    print_success(
        f"[4/4] Syncing install tree → {args.pi_target}:{args.pi_dir}/install …"
    )
    run(
        [
            "rsync",
            "-az",
            "--mkpath",
            "--delete",
            f"{ws_root / args.install_base}/",
            f"{args.pi_target}:{args.pi_dir}/install/",
        ]
    )

    # Sync runtime assets (launch files, configs, params, etc.) from src.
    print_success(
        "      Syncing runtime assets (launch / config / params / rviz / urdf / meshes) …"
    )
    run(
        [
            "rsync",
            "-az",
            "--mkpath",
            "--prune-empty-dirs",
            "--include=*/",
            "--include=launch/***",
            "--include=config/***",
            "--include=params/***",
            "--include=rviz/***",
            "--include=urdf/***",
            "--include=meshes/***",
            "--exclude=*",
            f"{ws_root}/src/",
            f"{args.pi_target}:{args.pi_dir}/src/",
        ]
    )

    print_success("Done.  Run on Pi:")
    print_success(f"  source /opt/ros/{args.ros_distro}/setup.bash")
    print_success(f"  source {args.pi_dir}/install/setup.bash")
    print_success("  ros2 launch <package> <launch_file>.launch.py")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
