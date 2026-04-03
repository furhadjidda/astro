#!/usr/bin/env python3
import argparse
import os
import re
import shlex
import subprocess
import sys
from pathlib import Path


def run(cmd: list[str], *, check: bool = True) -> int:
    print("+", " ".join(shlex.quote(x) for x in cmd))
    completed = subprocess.run(cmd)
    if check and completed.returncode != 0:
        raise SystemExit(completed.returncode)
    return completed.returncode


def ensure_cmd_exists(name: str) -> None:
    if (
        subprocess.run(
            ["bash", "-lc", f"command -v {shlex.quote(name)} >/dev/null 2>&1"]
        ).returncode
        != 0
    ):
        print(f"{name} is required", file=sys.stderr)
        raise SystemExit(1)


def arm64_emulation_ready() -> bool:
    cmd = [
        "docker",
        "run",
        "--rm",
        "--platform",
        "linux/arm64",
        "alpine:3.20",
        "uname",
        "-m",
    ]
    return (
        subprocess.run(
            cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
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


def find_workspace_root(start_dir: Path) -> Path:
    """Find nearest workspace root containing src/, preferring one with astro.repos."""
    fallback: Path | None = None
    for candidate in [start_dir, *start_dir.parents]:
        if (candidate / "src").is_dir():
            if (candidate / "astro.repos").is_file():
                return candidate
            if fallback is None:
                fallback = candidate

    if fallback is not None:
        return fallback

    raise SystemExit(
        "Could not find workspace root. Expected a parent directory containing 'src/'."
    )


def resolve_dockerfile(script_dir: Path) -> Path:
    for name in ("Dockerfile", "dockerfile"):
        candidate = script_dir / name
        if candidate.is_file():
            return candidate
    raise SystemExit(
        f"Dockerfile not found in {script_dir}. Expected one of: Dockerfile, dockerfile"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build ROS 2 workspace for Raspberry Pi 4 (ARM64) on host and optionally deploy."
    )
    mode_group = parser.add_mutually_exclusive_group()
    mode_group.add_argument(
        "--init",
        action="store_true",
        help="One-time setup: build deps image and exit",
    )
    mode_group.add_argument(
        "--quick",
        action="store_true",
        help="Fast dev loop preset (deps image + continue-on-error)",
    )
    mode_group.add_argument(
        "--full",
        action="store_true",
        help="Full refresh preset (clean + rebuild image)",
    )
    parser.add_argument(
        "--pi",
        dest="pi_target",
        default=os.environ.get("PI_TARGET", ""),
        help="SSH target for your Pi (example: ubuntu@192.168.1.120)",
    )
    parser.add_argument(
        "--pi-dir",
        default=os.environ.get("PI_DIR", "~/astro/ros2_ws"),
        help="Remote workspace path (default: ~/astro/ros2_ws)",
    )
    parser.add_argument(
        "--ros-distro",
        default=os.environ.get("ROS_DISTRO", "humble"),
        help="ROS distro in container (default: humble)",
    )
    parser.add_argument(
        "--image-name",
        default=os.environ.get("IMAGE_NAME", ""),
        help="Docker image name override",
    )
    parser.add_argument(
        "--deps-image",
        action="store_true",
        help="Use/build an image variant with rosdeps preinstalled from workspace src",
    )
    parser.add_argument(
        "--prepare-deps-image",
        action="store_true",
        help="Build deps image and exit (one-time setup)",
    )
    parser.add_argument(
        "--build-base", default=os.environ.get("BUILD_BASE", "build_pi4")
    )
    parser.add_argument(
        "--install-base", default=os.environ.get("INSTALL_BASE", "install_pi4")
    )
    parser.add_argument("--log-base", default=os.environ.get("LOG_BASE", "log_pi4"))
    parser.add_argument(
        "--openssl-root-dir",
        default=os.environ.get("OPENSSL_ROOT_DIR", "/usr"),
        help="OpenSSL root directory passed to CMake",
    )
    parser.add_argument(
        "--openssl-lib-dir",
        default=os.environ.get("OPENSSL_LIB_DIR", "/usr/lib/aarch64-linux-gnu"),
        help="OpenSSL library directory passed to CMake",
    )
    parser.add_argument(
        "--packages-up-to",
        default="",
        help="Pass package list to colcon --packages-up-to",
    )
    parser.add_argument(
        "--packages-select",
        default="",
        help="Pass package list to colcon --packages-select",
    )
    parser.add_argument(
        "--target",
        default="",
        help="Alias for --packages-select (space-separated package names)",
    )
    parser.add_argument(
        "--packages-skip",
        default=os.environ.get("PACKAGES_SKIP", "dynamixel_sdk_examples"),
        help="Pass package list to colcon --packages-skip",
    )
    parser.add_argument(
        "--clean-packages",
        default="",
        help="Space-separated package names to clean from build/install/log caches before build",
    )
    parser.add_argument(
        "--rosdep-skip-keys",
        default=os.environ.get(
            "ROSDEP_SKIP_KEYS", "micro_ros_agent realsense2_camera nav2_bringup"
        ),
        help="Space-separated rosdep keys to skip",
    )
    parser.add_argument(
        "--rosdep-continue-on-error",
        action="store_true",
        help="Continue even if some rosdep installs fail",
    )
    parser.add_argument(
        "--skip-image-build", action="store_true", help="Reuse existing Docker image"
    )
    parser.add_argument(
        "--rebuild-image",
        action="store_true",
        help="Force rebuilding Docker image even if it already exists",
    )
    parser.add_argument(
        "--skip-rosdep",
        action="store_true",
        help="Legacy flag (ignored): rosdep install/check is always run",
    )
    parser.add_argument(
        "--runtime-rosdep",
        action="store_true",
        help="Legacy flag (kept for compatibility): rosdep is always run",
    )
    parser.add_argument(
        "--force-rosdep-update",
        action="store_true",
        help="Always run rosdep update (otherwise uses existing cache when available)",
    )
    parser.add_argument(
        "--skip-deploy", action="store_true", help="Build only; do not rsync to Pi"
    )
    parser.add_argument(
        "--clean",
        action="store_true",
        help="Remove build/install/log directories for a clean rebuild before building",
    )
    parser.add_argument(
        "--continue-on-error",
        action="store_true",
        help="Pass through to colcon so unrelated packages continue building",
    )
    parser.add_argument(
        "--cmake-clean-cache",
        action="store_true",
        help="Pass through to colcon build to clear each package CMake cache",
    )
    parser.add_argument(
        "--no-sync-src-runtime",
        action="store_true",
        help="Do not sync launch/config/params/rviz/urdf/meshes from src",
    )
    parser.add_argument(
        "--setup-binfmt",
        action="store_true",
        help="Attempt one-time ARM64 emulation setup via docker",
    )
    parser.add_argument(
        "--import-repos",
        action="store_true",
        help="Run 'vcs import . < astro.repos' before build (CI-like)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    # Condensed mode presets for day-to-day usage.
    if args.init:
        args.prepare_deps_image = True
        args.deps_image = True
    if args.quick:
        args.deps_image = True
        args.continue_on_error = True
    if args.full:
        args.clean = True
        args.rebuild_image = True
        args.cmake_clean_cache = True
        args.rosdep_continue_on_error = True
        # args.runtime_rosdep = True  # Removed redundant rosdep mode toggle
        # args.skip_rosdep = False  # Removed redundant rosdep mode toggle

    if args.target.strip() and not args.packages_select.strip():
        args.packages_select = args.target

    if args.prepare_deps_image:
        args.deps_image = True

    script_dir = Path(__file__).resolve().parent
    ws_root = find_workspace_root(script_dir)
    dockerfile_path = resolve_dockerfile(script_dir)

    default_image_tag = (
        f"astro-ros2-pi4-builder:{args.ros_distro}-deps"
        if args.deps_image
        else f"astro-ros2-pi4-builder:{args.ros_distro}"
    )
    image_name = args.image_name or default_image_tag
    sync_src_runtime = not args.no_sync_src_runtime
    # Dependency installation/check is always enforced before build.
    effective_skip_rosdep = False

    if args.skip_rosdep:
        print("[warn] --skip-rosdep is ignored; dependency checks are always enforced")

    ensure_cmd_exists("docker")

    if not arm64_emulation_ready():
        print(
            "ARM64 container emulation is not configured (exec format error).",
            file=sys.stderr,
        )
        if args.setup_binfmt:
            print(
                "Attempting one-time setup: docker run --privileged --rm tonistiigi/binfmt --install arm64"
            )
            run(
                [
                    "docker",
                    "run",
                    "--privileged",
                    "--rm",
                    "tonistiigi/binfmt",
                    "--install",
                    "arm64",
                ]
            )
            if not arm64_emulation_ready():
                print(
                    "ARM64 emulation setup was attempted but is still failing.",
                    file=sys.stderr,
                )
                return 1
            print("ARM64 emulation configured successfully.")
        else:
            print("Run this once, then retry:", file=sys.stderr)
            print(
                "  docker run --privileged --rm tonistiigi/binfmt --install arm64",
                file=sys.stderr,
            )
            print("Or rerun this script with --setup-binfmt", file=sys.stderr)
            return 1

    if not args.skip_deploy and not args.pi_target:
        print("--pi is required unless --skip-deploy is used", file=sys.stderr)
        return 1

    image_exists = docker_image_exists(image_name)
    should_build_image = not args.skip_image_build and (
        args.rebuild_image or not image_exists
    )

    if should_build_image:
        print(f"[1/4] Building ARM64 image {image_name}")
        build_cmd = [
            "docker",
            "buildx",
            "build",
            "--platform",
            "linux/arm64",
            "--load",
            "--build-arg",
            f"ROS_DISTRO={args.ros_distro}",
            "--build-arg",
            f"PREINSTALL_ROSDEPS={'1' if args.deps_image else '0'}",
            "--build-arg",
            f"ROSDEP_SKIP_KEYS={args.rosdep_skip_keys}",
            "-t",
            image_name,
            "-f",
            str(dockerfile_path),
            str(ws_root),
        ]
        run(build_cmd)
    else:
        if args.skip_image_build:
            if not image_exists:
                print(
                    f"--skip-image-build was requested but image does not exist: {image_name}",
                    file=sys.stderr,
                )
                return 1
            print(f"[1/4] Skipping image build (using existing {image_name})")
        elif image_exists:
            print(
                f"[1/4] Reusing existing image {image_name} (pass --rebuild-image to rebuild)"
            )
        else:
            # Fallback guard; this branch should be unreachable with should_build_image logic.
            print(
                f"[1/4] Building image is required because {image_name} does not exist"
            )

    if args.prepare_deps_image:
        print("[2/4] Deps image preparation complete; exiting as requested")
        print(f"Use this image for fast builds: {image_name}")
        return 0

    if args.clean:
        print("[2/4] Cleaning build/install/log directories (default and pi4 trees)")
        run(
            [
                "docker",
                "run",
                "--rm",
                "--platform",
                "linux/arm64",
                "-v",
                f"{ws_root}:/ws",
                image_name,
                "bash",
                "-lc",
                (
                    f"rm -rf /ws/{args.build_base} /ws/{args.install_base} /ws/{args.log_base} "
                    "/ws/build /ws/install /ws/log"
                ),
            ]
        )

    if args.clean_packages.strip():
        pkgs = shlex.split(args.clean_packages)
        print(f"[2/4] Cleaning selected package caches: {' '.join(pkgs)}")
        # Remove only package-related artifacts to preserve incremental builds for the rest.
        cleanup_cmds = []
        for pkg in pkgs:
            cleanup_cmds.extend(
                [
                    f"rm -rf /ws/{args.build_base}/{pkg}*",
                    f"rm -rf /ws/build/{pkg}*",
                    f"rm -rf /ws/{args.log_base}/build_{pkg}*",
                    f"rm -rf /ws/log/build_*/{pkg}*",
                    f"rm -rf /ws/{args.install_base}/share/{pkg}",
                    f"rm -rf /ws/install/share/{pkg}",
                    f"rm -rf /ws/{args.install_base}/lib/{pkg}",
                    f"rm -rf /ws/install/lib/{pkg}",
                    f"rm -rf /ws/{args.install_base}/include/{pkg}",
                    f"rm -rf /ws/install/include/{pkg}",
                ]
            )
        run(
            [
                "docker",
                "run",
                "--rm",
                "--platform",
                "linux/arm64",
                "-v",
                f"{ws_root}:/ws",
                image_name,
                "bash",
                "-lc",
                " && ".join(cleanup_cmds),
            ]
        )

    container_steps: list[str] = [
        "set -e",
        f"source /opt/ros/{shlex.quote(args.ros_distro)}/setup.bash",
        f"export LD_LIBRARY_PATH=/opt/ros/{shlex.quote(args.ros_distro)}/lib:${{LD_LIBRARY_PATH:-}}",
        f"export CMAKE_PREFIX_PATH=/opt/ros/{shlex.quote(args.ros_distro)}:${{CMAKE_PREFIX_PATH:-}}",
        f"export CMAKE_LIBRARY_PATH=/opt/ros/{shlex.quote(args.ros_distro)}/lib:/usr/lib/aarch64-linux-gnu:${{CMAKE_LIBRARY_PATH:-}}",
    ]

    if args.import_repos:
        astro_repos = ws_root / "astro.repos"
        if astro_repos.exists():
            container_steps.append(
                "if [ -f astro.repos ]; then vcs import . < astro.repos; fi"
            )
        else:
            print(
                "[warn] --import-repos requested but astro.repos was not found at workspace root"
            )

    if not effective_skip_rosdep:
        rosdep_cmd = [
            "rosdep",
            "install",
            "--from-paths",
            "src",
            "utils",
            "--ignore-src",
            "-r",
            "-y",
            "--rosdistro",
            args.ros_distro,
        ]
        if args.rosdep_skip_keys.strip():
            rosdep_cmd.extend(["--skip-keys", args.rosdep_skip_keys])

        container_steps.extend(
            [
                "apt-get update -qq",
                (
                    f"apt-get install -y --no-install-recommends"
                    f" ros-{args.ros_distro}-builtin-interfaces"
                    f" ros-{args.ros_distro}-std-msgs"
                    f" ros-{args.ros_distro}-geometry-msgs"
                    f" ros-{args.ros_distro}-rcutils"
                    f" ros-{args.ros_distro}-vision-msgs"
                    f" ros-{args.ros_distro}-rosidl-default-generators"
                    f" ros-{args.ros_distro}-rosidl-default-runtime"
                    f" ros-{args.ros_distro}-rosidl-runtime-c"
                    f" ros-{args.ros_distro}-rosidl-typesupport-c"
                    f" libtinyxml2-dev"
                    f" libpython3-dev python3-dev python3.10-dev libpython3.10-dev python3-pip"
                ),
                "if ! dpkg -s python3-rosdep python3-vcstool >/dev/null 2>&1; then apt-get install -y --no-install-recommends python3-rosdep python3-vcstool; fi",
                "if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi",
                (
                    "rosdep update"
                    if args.force_rosdep_update
                    else "if [ ! -d /root/.ros/rosdep/sources.cache ]; then rosdep update; else echo '[rosdep] using existing cache (skip update; pass --force-rosdep-update to refresh)'; fi"
                ),
                shlex.join(rosdep_cmd)
                + (" || true" if args.rosdep_continue_on_error else ""),
                # Ensure unversioned .so symlinks exist for key ROS libraries.
                # Keep this targeted to avoid expensive scans under qemu emulation.
                (
                    f"for lib in builtin_interfaces__rosidl_generator_c "
                    f"builtin_interfaces__rosidl_typesupport_c "
                    f"builtin_interfaces__rosidl_typesupport_cpp rcutils; do "
                    f"for f in /opt/ros/{args.ros_distro}/lib/lib${{lib}}.so.*; do "
                    '[ -e "$f" ] || continue; '
                    'base="${f%%.so.*}.so"; '
                    '[ -e "$base" ] || ln -sf "$f" "$base"; '
                    "done; done"
                ),
            ]
        )
    else:
        if args.deps_image and not args.runtime_rosdep:
            print("[info] Skipping runtime rosdep because --deps-image is enabled")

    colcon_cmd = [
        "colcon",
        "--log-base",
        args.log_base,
        "build",
        "--base-paths",
        "src",
        "utils",
        "--merge-install",
        "--build-base",
        args.build_base,
        "--install-base",
        args.install_base,
    ]
    if args.packages_up_to.strip():
        colcon_cmd.extend(["--packages-up-to", *shlex.split(args.packages_up_to)])
    if args.packages_select.strip():
        colcon_cmd.extend(["--packages-select", *shlex.split(args.packages_select)])
    if args.packages_skip.strip():
        colcon_cmd.extend(["--packages-skip", *shlex.split(args.packages_skip)])
    colcon_cmd.append("--continue-on-error")
    if args.cmake_clean_cache:
        colcon_cmd.append("--cmake-clean-cache")
    colcon_cmd.extend(
        [
            "--cmake-args",
            "--no-warn-unused-cli",
            "-DCMAKE_BUILD_TYPE=Release",
            # Avoid ';' list separators here because the final command is executed
            # through bash -lc; ':' works for CMAKE_LIBRARY_PATH and is shell-safe.
            f"-DCMAKE_LIBRARY_PATH=/opt/ros/{args.ros_distro}/lib:/usr/lib/aarch64-linux-gnu",
            "-DPYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.10.so",
            "-DPYTHON_INCLUDE_DIR=/usr/include/python3.10",
            "-DPYTHON_LIBRARIES=/usr/lib/aarch64-linux-gnu/libpython3.10.so",
            "-DPYTHON_INCLUDE_DIRS=/usr/include/python3.10",
            "-DTINYXML2_LIBRARY=/usr/lib/aarch64-linux-gnu/libtinyxml2.so",
            "-DTINYXML2_INCLUDE_DIR=/usr/include",
            f"-Drcutils_DIR=/opt/ros/{args.ros_distro}/share/rcutils/cmake",
            f"-Dbuiltin_interfaces_DIR=/opt/ros/{args.ros_distro}/share/builtin_interfaces/cmake",
            f"-Dstd_msgs_DIR=/opt/ros/{args.ros_distro}/share/std_msgs/cmake",
            f"-Dgeometry_msgs_DIR=/opt/ros/{args.ros_distro}/share/geometry_msgs/cmake",
            f"-DOPENSSL_ROOT_DIR={args.openssl_root_dir}",
            f"-DOPENSSL_CRYPTO_LIBRARY={args.openssl_lib_dir}/libcrypto.so",
            f"-DOPENSSL_SSL_LIBRARY={args.openssl_lib_dir}/libssl.so",
        ]
    )
    container_steps.append(shlex.join(colcon_cmd))

    container_cmd = " && ".join(container_steps)

    print("[3/4] Building workspace for ARM64")
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
            container_cmd,
        ]
    )

    if args.skip_deploy:
        print("[3/4] Deploy skipped")
        print(f"Build output is ready at: {ws_root / args.install_base}")
        return 0

    if "@" in args.pi_target and args.pi_dir.startswith("/home/"):
        remote_user = args.pi_target.split("@", 1)[0]
        match = re.match(r"^/home/([^/]+)(/.*)?$", args.pi_dir)
        if match and match.group(1) != remote_user:
            print(
                "Remote deploy path user mismatch: "
                f"target user is '{remote_user}' but --pi-dir is under '/home/{match.group(1)}'.\n"
                "Use a writable path for the target user, for example:\n"
                f"  --pi-dir /home/{remote_user}/ros2_ws",
                file=sys.stderr,
            )
            return 1

    print(f"[4/4] Syncing install tree to {args.pi_target}:{args.pi_dir}/install")
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

    if sync_src_runtime:
        print(
            "[5/5] Syncing runtime assets from src (launch/config/params/rviz/urdf/meshes)"
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
    else:
        print("[5/5] Runtime src asset sync skipped")

    print("Done. Run on Pi with:")
    print(f"  source /opt/ros/{args.ros_distro}/setup.bash")
    print(f"  source {args.pi_dir}/install/setup.bash")
    print("  ros2 launch <your_package> <your_launch>.launch.py")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
