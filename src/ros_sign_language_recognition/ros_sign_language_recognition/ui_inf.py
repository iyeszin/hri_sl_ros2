'''
This code process each frame of the live video using the MediaPipe Pose model and save the landmark of each frame in a CSV file
Once stop the live video, it process the landmarks and put the result in the textbox
'''
import sys
import cv2
import csv
import numpy as np
import mediapipe as mp
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QTextEdit, QVBoxLayout, QWidget, QHBoxLayout, QLabel, QMessageBox
from PyQt5.QtGui import QImage, QPixmap
from std_msgs.msg import String
from ros_sign_language_recognition.predict import main as predict

import rclpy
from rclpy.node import Node


class VideoWidget(QWidget):
    def __init__(self, width, height):
        super().__init__()
        self.setMinimumSize(width, height)
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)
        self.label = QLabel('Live Video Placeholder')
        self.label.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(self.label)

    def update_frame(self, frame):
        # Convert BGR to RGB
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # Convert to QImage
        qimage = QImage(rgb_frame, frame.shape[1], frame.shape[0], QImage.Format_RGB888)
        # Convert QImage to QPixmap
        pixmap = QPixmap.fromImage(qimage)
        # Set the pixmap to the QLabel
        self.label.setPixmap(pixmap)


class ROS2UI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.init_ui()
        self.init_mediapipe()
        self.cap = cv2.VideoCapture(0)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.process_frame)
        self.frame_width = 640
        self.frame_height = 480
        self.fps = 30
        self.csv_writer = None
        self.recording = False
        self.frame_count = 0

        # Initialize ROS 2 node
        self.node = rclpy.create_node('ui_node')
        
        # Create publisher to publish gesture character
        self.publisher_ = self.node.create_publisher(String, 'gesture_char', 10)

    def init_ui(self):
        self.setWindowTitle("ROS2 UI")
        self.setGeometry(100, 100, 600, 400)

        # Buttons
        self.start_stop_button = QPushButton('Start Processing', self)
        self.start_stop_button.clicked.connect(self.toggle_processing)

        self.edit_done_button = QPushButton('Edit', self)
        self.edit_done_button.setDisabled(True)  # Disabled until processing starts
        self.edit_done_button.clicked.connect(self.edit_result)

        self.verify_button = QPushButton('Verify', self)
        self.verify_button.setDisabled(True)  # Disabled until processing starts
        self.verify_button.clicked.connect(self.verify_predict)

        # Placeholder for live video
        self.video_widget = VideoWidget(400, 450)

        # Text box for result
        self.result_textbox = QTextEdit()
        self.result_textbox.setReadOnly(True)

        # Layout
        layout = QVBoxLayout()
        layout.addWidget(self.video_widget)
        layout.addWidget(self.start_stop_button)

        # Horizontal layout for edit and done buttons
        button_layout = QHBoxLayout()
        button_layout.addWidget(self.edit_done_button)
        button_layout.addWidget(self.verify_button)
        layout.addLayout(button_layout)

        layout.addWidget(self.result_textbox)

        central_widget = QWidget()
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

    def init_mediapipe(self):
        mp_pose = mp.solutions.pose
        self.pose = mp_pose.Pose(static_image_mode=False, min_detection_confidence=0.5, min_tracking_confidence=0.5)
        mp_hands = mp.solutions.hands
        self.hands = mp_hands.Hands(static_image_mode=False, min_detection_confidence=0.5, min_tracking_confidence=0.5)
        mp_face_mesh = mp.solutions.face_mesh
        self.face = mp_face_mesh.FaceMesh(static_image_mode=True, max_num_faces=1, refine_landmarks=True, min_detection_confidence=0.5)

    def toggle_processing(self):
        self.recording = not self.recording
        if self.recording:
            self.start_stop_button.setText('Stop Processing')
            self.edit_done_button.setDisabled(True)
            self.verify_button.setDisabled(True)
            self.timer.start(1000 // self.fps)  # Adjust the timeout value as needed
            self.csv_writer = csv.writer(open('landmarks.csv', 'w'), delimiter=',', lineterminator='\n')

            header = ['frame']
            for part in ['face', 'left_hand', 'right_hand', 'pose']:
                if part == 'face':
                    landmark_count = 468
                elif part in ['left_hand', 'right_hand']:
                    landmark_count = 21
                else:  # Assuming the default is pose
                    landmark_count = 33

                for landmark in range(landmark_count):
                    for coord in ['x', 'y', 'z']:
                        header.append(f'{coord}_{part}_{landmark}')
            self.csv_writer.writerow(header)

        else:
            self.start_stop_button.setText('Start Processing')
            self.edit_done_button.setDisabled(False)
            self.verify_button.setDisabled(False)
            self.timer.stop()
            self.pose.close()
            self.hands.close()
            self.face.close()
            self.csv_writer = None
            QMessageBox.information(self, "Processing Finished", "Landmarks saved in landmarks.csv")
            self.process_predict()  # Automatically process landmarks once recording stops

    def edit_result(self):
        if self.edit_done_button.text() == 'Edit':
            self.result_textbox.setReadOnly(False)
            self.edit_done_button.setText('Done')
        else:
            self.result_textbox.setReadOnly(True)
            self.edit_done_button.setText('Edit')
            # Save or process the edited result if necessary
            result = self.result_textbox.toPlainText()
            print(f"Edited result: {result}")

    def process_frame(self):
        ret, frame = self.cap.read()
        if ret:
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            pose_results = self.pose.process(rgb_frame)
            hands_results = self.hands.process(rgb_frame)
            face_results = self.face.process(rgb_frame)
            if self.csv_writer:
                row = []

                if face_results.multi_face_landmarks:
                    for face_landmarks in face_results.multi_face_landmarks:
                        for landmark in face_landmarks.landmark:
                            x = landmark.x
                            y = landmark.y
                            z = landmark.z if hasattr(landmark, 'z') else 0
                            row.append(x)
                            row.append(y)
                            row.append(z)
                else:
                    row += [0] * (468 * 3)

                if hands_results.multi_hand_landmarks:
                    left_hand_landmarks = []
                    right_hand_landmarks = []

                    for hand_landmarks, handedness in zip(hands_results.multi_hand_landmarks, hands_results.multi_handedness):
                        if handedness.classification[0].label == 'Left':
                            left_hand_landmarks = hand_landmarks.landmark
                        else:
                            right_hand_landmarks = hand_landmarks.landmark

                    for landmark in left_hand_landmarks:
                        row.append(landmark.x)
                        row.append(landmark.y)
                        row.append(landmark.z if hasattr(landmark, 'z') else 0)

                    row += [0] * ((21 - len(left_hand_landmarks)) * 3)

                    for landmark in right_hand_landmarks:
                        row.append(landmark.x)
                        row.append(landmark.y)
                        row.append(landmark.z if hasattr(landmark, 'z') else 0)

                    row += [0] * ((21 - len(right_hand_landmarks)) * 3)
                else:
                    row += [0] * (21 * 3 * 2)

                if pose_results.pose_landmarks:
                    for idx, landmark in enumerate(pose_results.pose_landmarks.landmark):
                        row.append(landmark.x)
                        row.append(landmark.y)
                        row.append(landmark.z if hasattr(landmark, 'z') else 0)
                else:
                    row += [0] * 33 * 3

                self.csv_writer.writerow([self.frame_count] + row)

            self.frame_count += 1
            self.video_widget.update_frame(frame)

    def process_predict(self):
        print("Processing prediction...")
        result = predict()  # Call the main function from predict.py and capture the return value
        self.result_textbox.setText(result)  # Update the QTextEdit with the prediction result
        self.result_textbox.setReadOnly(True)
        self.verify_button.setDisabled(False)

    def verify_predict(self):
        print("Sending prediction to robot...")
        result = self.result_textbox.toPlainText()  # Get the result from the textbox
        print(f"Prediction result: {result}")
        
        # Publish the result to the 'gesture_char' topic
        msg = String()
        msg.data = result
        self.publisher_.publish(msg)

def main():
    # Initialize ROS 2 node
    rclpy.init()

    app = QApplication(sys.argv)
    ui = ROS2UI()
    ui.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
