#!/bin/bash

echo "Launching HMI Dashboard..."

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

python3 /ros2_ws/hmi/hmi_app.py

echo "HMI is running at: http://localhost:5000"
