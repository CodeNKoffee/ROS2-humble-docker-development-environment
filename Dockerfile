FROM ros:humble-ros-core

# Install basic utilities and ROS2 packages
RUN apt-get update && apt-get install -y \
  # Add these useful packages
  python3-pip \
  python3-colcon-common-extensions \
  ros-humble-turtlesim \
  ros-humble-rqt \
  ros-humble-rqt-common-plugins \
  # Add these for GUI support
  libxcb1-dev \
  libx11-xcb-dev \
  # Additional Qt dependencies
  qt5-default \
  qtbase5-dev \
  && rm -rf /var/lib/apt/lists/*

# Task 1: Configure ROS2 environment
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Task 11: Creating a workspace
WORKDIR /root/ros2_ws
RUN mkdir -p src

# Task 1: Configure ROS2 domain ID for node discovery
ENV ROS_DOMAIN_ID=0

# Task 1: Optional localhost restriction
# ENV ROS_LOCALHOST_ONLY=1

# Task 11: Source the workspace after building packages
RUN echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc