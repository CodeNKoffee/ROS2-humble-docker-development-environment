FROM ros:humble-ros-core

# Install basic utilities and ROS2 packages
RUN apt-get update && apt-get install -y \
  # Add these useful packages
  python3-pip \
  python3-colcon-common-extensions \
  ros-humble-turtlesim \
  ros-humble-rqt \
  ros-humble-rqt-common-plugins \
  ros-humble-demo-nodes-py \
  # Add these for GUI support
  libxcb1-dev \
  libx11-xcb-dev \
  libxcb-xinerama0 \
  # Additional Qt dependencies
  qtbase5-dev \
  qttools5-dev \
  libxcb1 \
  libx11-xcb1 \
  libxkbcommon-x11-0 \
  libxcb-icccm4 \
  libxcb-image0 \
  libxcb-keysyms1 \
  libxcb-randr0 \
  libxcb-render-util0 \
  libxcb-shape0 \
  libxcb-sync1 \
  libxcb-xfixes0 \
  libxcb-xinput0 \
  libxcb-xkb1 \
  # Install Xvfb for headless GUI rendering
  xvfb \
  x11vnc \
  && rm -rf /var/lib/apt/lists/*

# Task 1: Configure ROS2 environment
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Task 11: Creating a workspace
WORKDIR /root/ros2_ws
RUN mkdir -p src

# Task 1: Configure ROS2 domain ID for node discovery
ENV ROS_DOMAIN_ID=0

# Task 11: Source the workspace after building packages
RUN echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc