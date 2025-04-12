#!/bin/bash

echo "Launching all ROS 2 nodes..."

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

ros2 run barcode_scanner barcode_node &
ros2 run door_handle door_node &
ros2 run emergency_button emergency_node &
ros2 run stack_light stack_node &

echo "ROS 2 nodes are running."
echo "Press Ctrl+C to stop them."
wait
