# syntax=docker/dockerfile:1
ARG UBUNTU_VERSION=20.04
FROM ubuntu:${UBUNTU_VERSION}

ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=noetic
ARG USERNAME=ros
ARG UID=1000
ARG GID=1000

# Base tools (no pip ROS modules to avoid dpkg conflicts)
RUN apt-get update && apt-get install -y \
    locales tzdata sudo gnupg2 lsb-release curl wget ca-certificates \
    build-essential git vim nano bash-completion \
    python3-pip python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool \
 && locale-gen en_US.UTF-8 && update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 \
 && ln -fs /usr/share/zoneinfo/Etc/UTC /etc/localtime \
 && dpkg-reconfigure --frontend noninteractive tzdata

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# If any stray pip installs exist, remove them to prevent file collisions
RUN pip3 uninstall -y catkin_pkg rospkg rosdistro || true

# Add ROS apt repo via keyring and install
RUN mkdir -p /etc/apt/keyrings \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg \
 && sh -c 'echo "deb [signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1-latest.list' \
 && apt-get update \
 && apt-get -o Dpkg::Options::="--force-overwrite" install -y \
      ros-${ROS_DISTRO}-desktop-full \
      python3-catkin-tools \ 
      ros-${ROS_DISTRO}-rqt ros-${ROS_DISTRO}-rqt-common-plugins \
      ros-${ROS_DISTRO}-gazebo-ros-pkgs ros-${ROS_DISTRO}-gazebo-ros-control \
      mesa-utils libgl1-mesa-glx libx11-6 libxext6 libxrender1 libxrandr2 \
 && rm -rf /var/lib/apt/lists/*

# Optional NVIDIA runtime env (harmless if not used)
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all,compute,utility,graphics

# Create non-root user (no catkin_ws here)
RUN groupadd --gid ${GID} ${USERNAME} \
 && useradd -m -s /bin/bash --uid ${UID} --gid ${GID} ${USERNAME} \
 && usermod -aG sudo ${USERNAME} \
 && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/90-${USERNAME}

# ROS environment + entrypoint (only /opt/ros sourced)
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc \
 && printf '#!/usr/bin/env bash\nset -e\nsource /opt/ros/%s/setup.bash\nexec "$@"\n' "${ROS_DISTRO}" > /ros_entrypoint.sh \
 && chmod +x /ros_entrypoint.sh

# rosdep initialized at build for smoother first run
RUN rosdep init || true && rosdep update || true

USER ${USERNAME}
WORKDIR /home/

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
