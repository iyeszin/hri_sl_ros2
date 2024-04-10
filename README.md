# Human-Robot Interaction using Sign Language: A ROS2 based Framework
Sign language recognition is essential for enhancing communication with the deaf and hard-of-hearing community. This project integrates sign language recognition into the ROS2 framework for robotics applications.

## Installation
- Ubuntu Linux - Jammy Jellyfish (22.04)
- ROS 2 Humble
- Python 3.9 and above

## Prerequisites

Before you can use this package, ensure you have the following prerequisites installed and configured:

1. **ROS 2 Installation**:
   - Make sure you have ROS 2 installed. Visit the [ROS 2 website](https://docs.ros.org/en/humble/Installation.html) for installation instructions.
   - Check out also [Using colcon to build packages](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html).

2. **Python Dependencies**:
   - Install the required Python libraries:
     - `pip install serial`
     - `pip install pyserial`
     - `sudo apt install dbus-x11`
     <!-- Needed for run launch_gui.sh -->
     - `pip install pandas`
     <!-- Needed for run launch_all.sh -->
     - `pip install tensorflow`
     - `pip install keras`
     - `pip install mediapipe`
     
     

3. **USB Port Latency Timer**:
   - To ensure smooth operation, change the latency timer of the USB port. Run the following command in the terminal:
   
     ```console
     sudo gedit /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
     ```
     
     Set the value in this file to `1`.

4. **Add yourself into the 'dialout' group**:
    - First check if you are a member of that group:
    ```console
    groups ${USER}
    ```

    - If you don't belong to the dialout grup then add yourself to it:
    ```console
    sudo usermod -a -G dialout ${USER}
    ```


## Usage

1. Launch recognition module

    a. System Initialization:
    Ensure that your robotic arm and camera are connected and running.

    b. Camera Setup:
    Position a camera or sensor to capture the user's sign language gestures.

    c. Recognition Mode:
    Start the sign language recognition mode by running the script.

    ```shell
    ./launch_all.sh
    ```

    To run script, first give execute permission:
    ```shell
    chmod +x /path/to/yourscript.sh
    ```


    d. Recognition Start:
    Stand in front of the camera, ensuring that your sign language gestures are visible.

    e. Recognition and Action:
    The sign language recognition system will continuously analyze the gestures it captures from the camera feed. When it recognizes a specific gesture, it triggers the corresponding action on the robotic arm.

2. Launch gesture configuration GUI

    To find the position of every joints, a GUI can be launched to set the pre-trajetories for the hand.

    Prerequisite
    ```bash
    pip install pyqt5
    ```

    Run the script

    ```shell
    ./launch_gui.sh
    ```



## Features
List the key features of your project, such as:
- Real-time sign language recognition
- Integration with ROS 2 robotics applications
- Position joints for hand with GUI

## Training the Classifier

If you're interested in understanding how the sign language classifier was trained or want to contribute to the training process, please refer to our dedicated training repository. It contains detailed documentation, code, and data related to classifier training.

**Training Repository**: [Link to Classifier Training Repository](https://github.com/iyeszin/SignLanguageRecognition)


## Ros architecture
    ![ROS architecture](https://github.com/iyeszin/hri_sl_ros2/blob/main/ros_arch.png)

## The pipeline
    ![ROS architecture](https://github.com/iyeszin/hri_sl_ros2/blob/main/pipeline.gif)

Some results:
[Video results](
https://drive.google.com/drive/folders/1DWM2FxcwTKW34imyiGcpeCjsdzMU6Fxu?usp=sharing)

## Extra
If you mind so much on the deprecated warning, you can do the following:
1. set the PYTHONWARNINGS environment variable in your ~/.bashrc file
```bash
   export PYTHONWARNINGS="ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::pkg_resources"
```
2. update every setup.cfg
from script-dir=... to script_dir=...
from install-scripts=... to install_scripts=...

## Contributing
We welcome contributions from the community! If you'd like to contribute to this project, please follow our contribution guidelines.

## Contact
If you have questions or need support, you can contact us at iyeszin@gmail.com.