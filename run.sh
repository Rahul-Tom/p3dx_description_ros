#!/bin/bash

echo "Script execution started."

# Launch p3dx_description_ros (Pioneer 3DX description) in the background
ros2 launch p3dx_description_ros p3dx_description_ros2.launch.py &
pid1=$!
if [ $? -ne 0 ]; then
  echo "Failed to launch p3dx_description_ros. Exiting..."
  exit 1
fi

# Delay for 1 second before launching the next process
sleep 1

# Launch the navigation2 localization with a custom map in the background
ros2 launch nav2_bringup localization_launch.py map:=./maps/RoboticsLab_1309_1529.yaml &
pid2=$!
if [ $? -ne 0 ]; then
  echo "Failed to launch localization. Exiting..."
  exit 1
fi

# Delay for 1 second before launching the next process
sleep 1

# Launch navigation for p3dx_description_ros in the background
ros2 launch p3dx_description_ros navigation_launch.py &
pid3=$!
if [ $? -ne 0 ]; then
  echo "Failed to launch navigation for p3dx_description_ros. Exiting..."
  exit 1
fi

# Wait for all background processes to complete
wait $pid1 $pid2 $pid3

echo "Script execution completed."
