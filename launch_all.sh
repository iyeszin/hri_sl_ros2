# 1. Build the ROS 2 workspace:
colcon build
. install/setup.bash

# 2. Launch the required components in separate terminals:
ros2 launch rh8d_controller_python position_controller.launch.py
ros2 launch rh8d_hw rh8d_hw_combined.launch.py
ros2 launch rh8d_ros2 rh8d_col.launch.py
ros2 launch ros_sign_language_recognition recognizer_launch.py
ros2 launch ros_chat_agent chat_agent.launch.py
