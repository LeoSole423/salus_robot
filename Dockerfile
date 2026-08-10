FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-pip \
    python3-pytest \
    python3-serial \
    python3-rosdep \
    python3-vcstool \
    ros-humble-ament-cmake \
    ros-humble-ament-cmake-pytest \
    ros-humble-ament-lint-auto \
    ros-humble-ament-lint-common \
    ros-humble-launch \
    ros-humble-launch-ros \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-sensor-msgs \
    ros-humble-std-msgs \
    && rm -rf /var/lib/apt/lists/*

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && mkdir -p /ros2_ws \
    && chown -R ${USERNAME}:${USERNAME} /ros2_ws

COPY entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

USER ${USERNAME}
WORKDIR /ros2_ws
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
