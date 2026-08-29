FROM osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# libgz-sim8-dev (needed to build auv_current_plugin against gz-sim headers)
# lives in the Gazebo project's own apt repo, not packages.ros.org -- add it
# before installing anything from it.
RUN apt-get update && apt-get install -y --no-install-recommends \
    lsb-release wget gnupg \
    && wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
       > /etc/apt/sources.list.d/gazebo-stable.list

# Gazebo Harmonic (the Jazzy-paired Gazebo release) + ROS<->Gazebo bridge,
# Nav2, robot_localization for sensor fusion, and dev tooling.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros-gz \
    libgz-sim8-dev \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-pluginlib \
    ros-${ROS_DISTRO}-nav2-core \
    ros-${ROS_DISTRO}-nav2-costmap-2d \
    ros-${ROS_DISTRO}-nav2-util \
    ros-${ROS_DISTRO}-nav2-msgs \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    cmake \
    git \
    && rm -rf /var/lib/apt/lists/*

RUN if [ ! -d /etc/ros/rosdep/sources.list.d ] || [ -z "$(ls -A /etc/ros/rosdep/sources.list.d 2>/dev/null)" ]; then rosdep init; fi

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && apt-get update && apt-get install -y --no-install-recommends sudo && rm -rf /var/lib/apt/lists/*

USER $USERNAME
RUN rosdep update

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "if [ -f /workspaces/pralay2_nav_sim/install/setup.bash ]; then source /workspaces/pralay2_nav_sim/install/setup.bash; fi" >> /home/$USERNAME/.bashrc

WORKDIR /workspaces/pralay2_nav_sim
