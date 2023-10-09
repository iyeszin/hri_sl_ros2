# Packages in the project
1. rh8d_hw
- it contains all the hardware interfaces of the 
hand. 
- two functionalities are executed by the package, on the one hand the reading and publishing of the tactile sensor data, on the other hand the basic control of the hand and the reading of the joint data.

2. rh8d_msgs
- it provides all the required custom messages.
- at CMakeLists.txt, action, srv, msg needed to place in ascending order.

3. rh8d_ros2 
- it contains sample code for the collector and playback programs.

4. rh8d_controller
- it contains sample code for the controller to run random target positions and speeds at every joints
- it contains sample code for the controller to run pre-defined target positions and speed at every joint
- it contains code for the controller to read the file with pre-recorded trajectories instead of taking hard code trajectories
- it contains code to run the GUI to find the target positions of every joint variable

5. ros_chat_agent
- it contains code that act as a middleman which receiving and sending requests

6. ros_sign_language_recognition
- it provides the inference code to perform recognition on input image and output the result of sign language gesture

