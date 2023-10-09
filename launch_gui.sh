# 1. Build the ROS 2 workspace:
colcon build
. install/setup.bash

# 2. Launch 
ros2 launch rh8d_controller_python position_controller.launch.py 