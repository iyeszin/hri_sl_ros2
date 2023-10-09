from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QSlider, QPushButton
from PyQt5.QtCore import Qt
import math
import rclpy
from rclpy.node import Node
from rh8d_msgs.msg import JointSetSpeedPos, JointListSetSpeedPos
import sys
import threading
import time

class RH8DSampleController(Node):
    def __init__(self):
        self.name = "rh8d_sample_controller"
        super().__init__(self.name + "_node")

        self.declare_parameters(
            namespace='',
            parameters=[
                ('hand_polarity', 'L_'),
                ('frequency', 50)
            ]
        )

        try:
            self.polarity = self.get_parameter('hand_polarity').value
            self.get_logger().info("Hand polarity is : %s" % self.polarity)
        except KeyError:
            self.get_logger().info("Could not find hand polarity parameter, setting to default (L_)")
            pass

        try:
            self.frequency = self.get_parameter('frequency').value
            self.get_logger().info("Hand polarity is : %s" % self.frequency)
        except KeyError:
            self.get_logger().info("Could not find frequency parameter, setting to default (50)")
            pass

        # Initialize the publisher and timer objects
        self.joint_pub = self.create_publisher(JointListSetSpeedPos, self.polarity + 'setSpeedPos', 10)

        # self.timer_period = 1 / self.frequency
        # self.main_loop_sub = self.create_timer(self.timer_period, self.mainLoopCallback)

        # create msg object
        self.lone_joints = [JointSetSpeedPos() for i in range(8)]
        self.msg = JointListSetSpeedPos()

        # joint variables
        self.joint_names = [self.polarity.lower() + 'w_rotation',
                            self.polarity.lower() + 'w_flexion',
                            self.polarity.lower() + 'w_adduction',
                            self.polarity.lower() + 'th_adduction',
                            self.polarity.lower() + 'th_flexion',
                            self.polarity.lower() + 'ix_flexion',
                            self.polarity.lower() + 'middle_flexion',
                            self.polarity.lower() + 'ring_ltl_flexion']

        self.target_position_list = [2048, 2048, 2048, 0, 0, 0, 0, 0]
        self.target_speed_list = [0, 0, 0, 0, 0, 0, 0, 0]

        # ticker
        self.t = 0

    def sliderValueChanged(self, slider_index, value):
        # Update the target position for the corresponding joint based on the slider value
        self.target_position_list[slider_index] = value

        # Publish the updated joint positions
        self.publishJointPositions()

    def publishJointPositions(self):
        # Publish the joint positions
        for name, position, speed, joint in zip(self.joint_names, self.target_position_list, self.target_speed_list, self.lone_joints):
            joint.name = name
            joint.target_pos = position
            joint.target_speed = speed

        self.msg.joints = self.lone_joints
        self.joint_pub.publish(self.msg)

    def initPosition(self):
        self.target_position_list = [2048, 2048, 2048, 0, 0, 0, 0, 0]
        self.target_speed_list = [0, 0, 0, 0, 0, 0, 0, 0]

        for name, position, speed, joint in zip(self.joint_names, self.target_position_list, self.target_speed_list, self.lone_joints):
            joint.name = name
            joint.target_pos = position
            joint.target_speed = speed

        self.msg.joints = self.lone_joints
        self.joint_pub.publish(self.msg)

class RH8DControllerGUI(QMainWindow):
    def __init__(self, controller_node):
        super().__init__()

        self.controller_node = controller_node
        self.sliders = []
        self.slider_labels = []
        self.value_labels = []
        self.input_value = 0
        self.joint_name = ""

        self.initUI()

    def initUI(self):
        self.setWindowTitle('RH8D Controller GUI')

        central_widget = QWidget(self)
        layout = QVBoxLayout(central_widget)

        sliders_layout = QVBoxLayout()  # Vertical layout for sliders

        for i in range(len(self.controller_node.joint_names)):
            label = QLabel(self.controller_node.joint_names[i])
            sliders_layout.addWidget(label)
            self.slider_labels.append(label)

            value_label = QLabel('0')
            sliders_layout.addWidget(value_label)
            self.value_labels.append(value_label)

            slider = QSlider(Qt.Horizontal)
            slider.setRange(0, 4095)
            slider.valueChanged.connect(lambda value, index=i: self.controller_node.sliderValueChanged(index, value))
            slider.valueChanged.connect(lambda value, index=i: self.sliderValueChanged(index, value))
            sliders_layout.addWidget(slider)

            self.sliders.append(slider)
            

        # Buttons
        buttons_layout = QHBoxLayout()  # Horizontal layout for buttons

        self.button_reset = QPushButton("Reset", self)
        self.button_reset.clicked.connect(self.resetClicked)
        buttons_layout.addWidget(self.button_reset)

        self.button_save = QPushButton("Save", self)
        self.button_save.clicked.connect(self.saveClicked)
        buttons_layout.addWidget(self.button_save)

        layout.addLayout(sliders_layout)
        layout.addLayout(buttons_layout)

        self.setCentralWidget(central_widget)

    def sliderValueChanged(self, index, value):
        self.value_labels[index].setText(str(value))  # Update value label

    def resetSliders(self):
        initial_value = 0  # Set the desired initial value for the sliders
        for slider, label in zip(self.sliders, self.value_labels):
            slider.setValue(initial_value)
            label.setText(str(initial_value))  # Update value label

    def resetClicked(self):
        self.resetSliders()

    def saveClicked(self):
        self.joint_name = self.comboBox.currentText()
        print("Saved Joint Name:", self.joint_name)
        print("Saved Value:", self.input_value)

def run_ros2_node(controller_node):
    rclpy.spin(controller_node)

def main(args=None):
    rclpy.init(args=args)

    app = QApplication(sys.argv)  # Create QApplication instance here

    # Create an instance of the controller node
    controller_node = RH8DSampleController()

    # Create an instance of the GUI and pass the controller node
    controller_gui = RH8DControllerGUI(controller_node)


    try:
        controller_gui.show()

        # Create and start a separate thread to run the ROS2 event loop
        ros2_thread = threading.Thread(target=run_ros2_node, args=(controller_node,))
        ros2_thread.start()

        # Start the PyQt5 event loop
        sys.exit(app.exec_())

    finally:
        controller_node.initPosition()
        time.sleep(2)
        controller_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
