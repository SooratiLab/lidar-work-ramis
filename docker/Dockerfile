# ROS 2 Humble image for the Go2 Mid-360 LiDAR stack: livox_ros_driver2 +
# FAST_LIO_ROS2. Replaces the manual, many-pitfalls build-from-scratch
# workflow inherited from ros2-go2 with a reproducible image.
#
# Built and tested on x86_64 as a stand-in for the Jetson's aarch64 -- ROS's
# own Humble base images and every upstream repo cloned below support both
# architectures. The aarch64 build and runtime still need live Jetson
# verification; see docker/README.md's first-live-run checklist.

FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git \
      libpcl-dev libeigen3-dev libapr1-dev \
      ros-humble-pcl-conversions ros-humble-pcl-ros \
      ros-humble-rmw-cyclonedds-cpp \
      ros-humble-rosidl-generator-dds-idl \
      python3-colcon-common-extensions python3-rosdep \
      gettext-base \
      python3-numpy python3-scipy python3-sklearn \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# Livox-SDK2 -- built and installed system-wide, outside the colcon
# workspace. If it ends up under ros2_ws/src instead, colcon tries to build
# it as a ROS package alongside livox_ros_driver2 and both builds fail. This
# reproduces a failure already encountered in the inherited Foxy setup.
# ---------------------------------------------------------------------------
RUN git clone --depth 1 https://github.com/Livox-SDK/Livox-SDK2.git /opt/Livox-SDK2 \
    && cmake -S /opt/Livox-SDK2 -B /opt/Livox-SDK2/build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /opt/Livox-SDK2/build --parallel "$(nproc)" \
    && cmake --install /opt/Livox-SDK2/build \
    && ldconfig

# ---------------------------------------------------------------------------
# ROS workspace: livox_ros_driver2 + FAST_LIO_ROS2 (Ericsii's ROS2 fork --
# the official hku-mars/FAST_LIO is ROS1-only and depends on the ROS1-only
# livox_ros_driver, not livox_ros_driver2).
# ---------------------------------------------------------------------------
WORKDIR /opt/ros2_ws/src

RUN git clone --depth 1 https://github.com/Livox-SDK/livox_ros_driver2.git \
    && cp livox_ros_driver2/package_ROS2.xml livox_ros_driver2/package.xml

# Default branch is "ros2" -- pin it explicitly rather than relying on
# whatever the default happens to be at clone time.
RUN git clone --recursive --depth 1 --branch ros2 \
      https://github.com/Ericsii/FAST_LIO_ROS2.git

# Unitree's small request-message package is needed only by the explicitly
# enabled StopMove adapter. Fetch the exact revision mirrored under sdks/ in
# the parent workspace rather than letting a future upstream change alter the
# wire definition silently.
ARG UNITREE_ROS2_COMMIT=668d1ec5a05d1c38d3306bdca7d59f2ba3581a88
RUN git init /tmp/unitree_ros2 \
    && git -C /tmp/unitree_ros2 remote add origin \
         https://github.com/unitreerobotics/unitree_ros2.git \
    && git -C /tmp/unitree_ros2 fetch --depth 1 origin "${UNITREE_ROS2_COMMIT}" \
    && git -C /tmp/unitree_ros2 checkout FETCH_HEAD \
    && cp -a /tmp/unitree_ros2/cyclonedds_ws/src/unitree/unitree_api \
         /opt/ros2_ws/src/unitree_api \
    && rm -rf /tmp/unitree_ros2

# LiDAR timestamp fix -- see patch_livox_timestamp.py for the full
# explanation. Needed on every ROS distro; it's a Mid-360 firmware quirk,
# not a Foxy-vs-Humble difference.
COPY patch_livox_timestamp.py /tmp/patch_livox_timestamp.py
RUN python3 /tmp/patch_livox_timestamp.py /opt/ros2_ws/src/livox_ros_driver2

# The inherited ConstSharedPtr -> SharedPtr laserMapping.cpp patch is a
# Foxy-only workaround: Ericsii's fork is written against Humble's
# AnyServiceCallback API (ConstSharedPtr), which Foxy's older API doesn't
# accept. On Humble the upstream source is already correct as cloned --
# deliberately not patching it here.

RUN rosdep init 2>/dev/null || true && rosdep update --rosdistro humble
# apt's package index was dropped after the first apt-get layer above to
# keep image size down -- rosdep's apt backend needs it back, or every
# lookup fails with "Unable to locate package" regardless of whether the
# package actually exists.
RUN apt-get update \
    && source /opt/ros/humble/setup.bash \
    && rosdep install --from-paths . --ignore-src -r -y --rosdistro humble \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/ros2_ws
RUN source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install --parallel-workers "$(nproc)" \
         --cmake-args -DCMAKE_BUILD_TYPE=Release -DDISTRO_ROS=humble

# ---------------------------------------------------------------------------
# Runtime configuration
# ---------------------------------------------------------------------------
COPY config/fastrtps_eth0_only.xml /opt/fastrtps_eth0_only.xml
COPY config/cyclonedds_eth0.xml /opt/cyclonedds_eth0.xml
COPY config/MID360_config.json.template /opt/MID360_config.json.template
COPY entrypoint.sh /opt/entrypoint.sh
RUN chmod +x /opt/entrypoint.sh

# CycloneDDS matches Unitree's own ROS 2 stack and is now the default test
# path on Humble. Kei's node-creation crash was in the host's Foxy install,
# so it should not be carried forward as an assumption about this image.
# docker-compose.yml exposes RMW_IMPLEMENTATION for an immediate Fast DDS
# fallback if the live Jetson test finds a Humble-specific problem.
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///opt/cyclonedds_eth0.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/opt/fastrtps_eth0_only.xml

ENTRYPOINT ["/opt/entrypoint.sh"]
CMD ["driver"]
