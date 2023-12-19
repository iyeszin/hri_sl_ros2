#!/bin/bash

# Build the ROS 2 workspace
colcon build
source install/setup.bash

# Launch the ROS 2 nodes
gnome-terminal -- bash -c "ros2 launch rh8d_hw hw_without_tactile.launch.py; exec bash" --hold
gnome-terminal -- bash -c "ros2 launch rh8d_ros2 rh8d_col.launch.py; exec bash" --hold
gnome-terminal -- bash -c "ros2 launch rh8d_controller_python controller_gui.launch.py; exec bash" --hold
