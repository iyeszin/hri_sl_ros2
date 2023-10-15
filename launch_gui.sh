# 1. Build the ROS 2 workspace:
colcon build
. install/setup.bash

# 2. Launch 
ros2 launch rh8d_hw rh8d_hw_combined.launch.py
ros2 launch rh8d_ros2 rh8d_col.launch.py
ros2 launch rh8d_controller_python controller_gui.launch.py
