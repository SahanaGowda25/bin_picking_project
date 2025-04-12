#!/bin/bash

echo "Launching Robot Client + WMS API..."

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

python3 /ros2_ws/api/robot_client.py &
sleep 2
python3 /ros2_ws/api/wms_server.py &

echo "APIs are running."
echo "WMS: http://localhost:8080"
echo "Robot Client: http://localhost:8081"
echo "Press Ctrl+C to stop them."
wait
