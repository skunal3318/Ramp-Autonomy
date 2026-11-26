# Multi-stage Dockerfile for Ramp Autonomy
FROM ubuntu:24.04 AS base
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update && apt-get install -y \
    wget \
    gnupg \
    lsb-release \
    software-properties-common \
    ca-certificates \
    curl \
    git \
    build-essential \
    cmake \
    python3 \
    python3-pip \
    python3-venv \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-rosinstall-generator \
    python3-wstool \
    libxtensor-dev \
    libxtensor-python-dev \
    libtinyxml2-dev \
    libeigen3-dev \
    freeglut3-dev \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    net-tools \
    iproute2 \
    && rm -rf /var/lib/apt/lists/*

# Add ROS 2 repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Jazzy
RUN apt-get update && apt-get install -y \
    ros-jazzy-desktop \
    ros-jazzy-gazebo-ros2-control \
    ros-jazzy-gazebo-ros-pkgs \
    ros-jazzy-nav2 \
    ros-jazzy-robot-localization \
    && rm -rf /var/lib/apt/lists/*

# Set up environment
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
ENV ROS_DISTRO= jazzy

# Create workspace
RUN mkdir -p /workspace/ramp_autonomy_ws/src
WORKDIR /workspace/ramp_autonomy_ws

# Copy source code
COPY . src/

# Pre-warm Gazebo Fuel cache
RUN . /opt/ros/jazzy/setup.bash && \
    gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so worlds/construction_site.sdf & \
    GAZEBO_PID=$! && \
    sleep 30 && \
    kill $GAZEBO_PID || true

# Build workspace
RUN . /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install --packages-select ramp_autonomy_bringup ramp_autonomy_control ramp_autonomy_critics ramp_autonomy_description ramp_autonomy_gazebo ramp_autonomy_msgs ramp_autonomy_nav ramp_autonomy_perception

# Final runtime stage
FROM base AS runtime
WORKDIR /workspace/ramp_autonomy_ws

# Copy built workspace
COPY --from=base /workspace/ramp_autonomy_ws /workspace/ramp_autonomy_ws

# Source setup in entrypoint
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /workspace/ramp_autonomy_ws/install/setup.bash" >> /root/.bashrc

# Create entrypoint script
COPY <<'EOF' /entrypoint.sh
#!/bin/bash
set -e

# Source ROS environment
source /opt/ros/jazzy/setup.bash
source /workspace/ramp_autonomy_ws/install/setup.bash

# Handle X11 forwarding for GUI
if [ "$1" = "--gui" ]; then
    echo "Starting with GUI support"
    # Set up X11 forwarding
    export DISPLAY=:0
    # Start X server if not running
    if ! pgrep -x "Xorg" > /dev/null; then
        Xorg :0 -ac -noreset -silent-dpi 100 &
        sleep 2
    fi
    # Start Gazebo with GPU support
    export GAZEBO_PLUGIN_PATH=/opt/ros/jazzy/lib/gazebo_plugins:
    export GAZEBO_MODEL_PATH=/opt/ros/jazzy/share/gazebo_ros_models:/workspace/ramp_autonomy_ws/src/ramp_autonomy_gazebo/worlds/models
    exec "$@"
else
    echo "Starting in headless mode"
    # Set headless flag for Gazebo
    export GAZEBO_HEADLESS=true
    exec "$@"
fi
EOF

RUN chmod +x /entrypoint.sh

# Default to headless mode
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "ramp_autonomy_bringup", "ramp_autonomy_bringup.launch.py", "world:=construction_site", "payload_mass:=100.0", "headless:=true"]
