import sys
import time
import math
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QSlider, QLabel
from PyQt5.QtCore import QTimer
from PyQt5.QtCore import Qt


class RH8DSampleController:
    def __init__(self):
        self.joint_names = ['w_rotation', 'w_flexion', 'w_adduction', 'th_adduction',
                            'th_flexion', 'ix_flexion', 'middle_flexion', 'ring_ltl_flexion']

        self.target_position_list = [2048, 2048, 2048, 0, 0, 0, 0, 0]
        self.target_speed_list = [0, 0, 0, 0, 0, 0, 0, 0]

        # ticker
        self.t = 0

    def mainLoopCallback(self):
        sine = math.sin(self.t / 8)
        self.target_position_list[4] = int(3000 * sine)
        self.target_position_list[5] = int(3000 * sine)
        self.target_position_list[6] = int(3000 * sine)
        self.target_position_list[7] = int(3000 * sine)

        # Publish target positions and speeds to the robot

        if self.t < 200:
            self.t += 1
        else:
            self.t = 0

    def initPosition(self):
        self.target_position_list = [2048, 2048, 2048, 0, 0, 0, 0, 0]
        self.target_speed_list = [0, 0, 0, 0, 0, 0, 0, 0]


class RH8DControllerGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("RH8D Controller")
        self.controller = RH8DSampleController()
        self.setupUI()

    def setupUI(self):
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout()

        self.sliders = []
        self.labels = []

        for i, joint_name in enumerate(self.controller.joint_names):
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(0)
            slider.setMaximum(4095)
            slider.setTickInterval(100)
            slider.setTickPosition(QSlider.TicksBelow)
            slider.valueChanged.connect(lambda value, index=i: self.sliderValueChanged(index, value))
            self.sliders.append(slider)

            label = QLabel(joint_name)
            self.labels.append(label)

            self.layout.addWidget(label)
            self.layout.addWidget(slider)

        self.central_widget.setLayout(self.layout)

    def sliderValueChanged(self, index, value):
        self.controller.target_position_list[index] = value


def main():
    app = QApplication(sys.argv)
    controller_gui = RH8DControllerGUI()
    controller_gui.show()

    timer = QTimer()
    timer.timeout.connect(controller_gui.controller.mainLoopCallback)
    timer.start(50)  # Update every 50 milliseconds (20 Hz)

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
