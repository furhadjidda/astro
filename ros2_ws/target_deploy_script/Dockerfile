ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}

ENV DEBIAN_FRONTEND=noninteractive

# System build tools and libraries.
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libyaml-dev \
    libssl-dev \
    libtinyxml2-dev \
    libpython3-dev \
    python3-dev \
    python3.10-dev \
    libpython3.10-dev \
    libusb-1.0-0-dev \
    libudev-dev \
    libopencv-dev \
    nlohmann-json3-dev \
    libspdlog-dev \
    libfmt-dev \
    pkg-config \
    curl \
    wget \
    zip \
    unzip \
    tar \
    # ROS build and package management tools \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    rsync \
    # ROS packages required by workspace source packages \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-yaml-cpp-vendor \
    ros-${ROS_DISTRO}-turtlebot3-msgs \
    # depthai ROS driver (v3) – installed from apt so depthai-core and
    # vision_msgs are NOT built from source; this eliminates the need to
    # clone depthai-core / depthai-ros / vision_msgs via vcs.
    # On Humble/Jazzy the packages carry the _v3 suffix.
    # If apt cannot find the package, enable the ROS testing repo first:
    #   echo "deb http://packages.ros.org/ros-testing/ubuntu jammy main" \
    #        > /etc/apt/sources.list.d/ros-testing.list
    ros-${ROS_DISTRO}-depthai-ros-v3 \
    && rm -rf /var/lib/apt/lists/*

# Intel RealSense SDK 2.0 (librealsense2).
# Required to build realsense2_camera from source (utils/realsense_ros).
# The SDK is not in the standard Ubuntu repos; it comes from Intel's own apt repo.
# We fetch the signing key by its exact fingerprint from the Ubuntu keyserver
# because the .pgp file Intel hosts does not match the key used to sign the repo.
RUN apt-get update && apt-get install -y --no-install-recommends gpg gpg-agent \
    && mkdir -p /etc/apt/keyrings \
    && curl -sSL "https://keyserver.ubuntu.com/pks/lookup?op=get&search=0xFB0B24895113F120" \
       | gpg --dearmor -o /etc/apt/keyrings/librealsense.gpg \
    && echo "deb [signed-by=/etc/apt/keyrings/librealsense.gpg] \
       https://librealsense.intel.com/Debian/apt-repo jammy main" \
       > /etc/apt/sources.list.d/librealsense.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
       librealsense2-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
