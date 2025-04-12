FROM ros:humble

# Install system dependencies
RUN apt update && apt install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-rclpy \
    ros-humble-std-msgs \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages for API handling
RUN pip3 install flask fastapi uvicorn requests

# Set up ROS workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Set default shell
SHELL ["/bin/bash", "-c"]

# Source ROS setup for future commands
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
