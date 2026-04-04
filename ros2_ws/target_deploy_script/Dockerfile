ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}
ARG PREINSTALL_ROSDEPS=0
ARG ROSDEP_SKIP_KEYS=micro_ros_agent

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    libc6-dev \
    cmake \
    git \
    libyaml-dev \
    libssl-dev \
    libtinyxml2-dev \
    libpython3-dev \
    python3-dev \
    python3.10-dev \
    libpython3.10-dev \
    ros-${ROS_DISTRO}-builtin-interfaces \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-rcutils \
    ros-${ROS_DISTRO}-vision-msgs \
    ros-${ROS_DISTRO}-rosidl-default-generators \
    ros-${ROS_DISTRO}-rosidl-default-runtime \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    rsync \
    && rm -rf /var/lib/apt/lists/*

# Optional one-time dependency baking from workspace source tree.
COPY src /tmp/ws/src
COPY utils /tmp/ws/utils
RUN if [ "${PREINSTALL_ROSDEPS}" = "1" ]; then \
            apt-get update && \
            if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi && \
            rosdep update && \
            rosdep install --from-paths /tmp/ws/src /tmp/ws/utils --ignore-src -r -y --rosdistro ${ROS_DISTRO} --skip-keys "${ROSDEP_SKIP_KEYS}"; \
        fi && \
        rm -rf /tmp/ws && \
        rm -rf /var/lib/apt/lists/*

# Keep container simple; workspace is mounted at runtime.
WORKDIR /ws
