#!/bin/bash

# Build the ROS 2 workspace
colcon build
source install/setup.bash

# Launch the ROS 2 nodes
gnome-terminal -- bash -c "source install/setup.bash; ros2 launch rh8d_controller_python position_controller.launch.py; exec bash" --hold
gnome-terminal -- bash -c "source install/setup.bash; ros2 launch rh8d_hw rh8d_hw_combined.launch.py; exec bash" --hold
gnome-terminal -- bash -c "source install/setup.bash; ros2 launch rh8d_ros2 rh8d_col.launch.py; exec bash" --hold
gnome-terminal -- bash -c "source install/setup.bash; ros2 launch ros_sign_language_recognition recognizer_launch.py; exec bash" --hold
gnome-terminal -- bash -c "source install/setup.bash; ros2 launch ros_chat_agent chat_agent.launch.py; exec bash" --hold
