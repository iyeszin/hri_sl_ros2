# main inference file
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from cv_bridge import CvBridge
import cv2 as cv
import copy
import numpy as np
import mediapipe as mp
from keras import models
from collections import deque
from rclpy.node import Node
from rh8d_msgs.srv import CustomService
import threading
import time

class_names = ["A","B","C","D","E","F","G","H","I","K","L",'M','N','O','P','Q','R','S','T','U','V','W','X','Y']

class CvFpsCalc(object):
    def __init__(self, buffer_len=1):
        self._start_tick = cv.getTickCount()
        self._freq = 1000.0 / cv.getTickFrequency()
        self._difftimes = deque(maxlen=buffer_len)

    def get(self):
        current_tick = cv.getTickCount()
        different_time = (current_tick - self._start_tick) * self._freq
        self._start_tick = current_tick

        self._difftimes.append(different_time)

        fps = 1000.0 / (sum(self._difftimes) / len(self._difftimes))
        fps_rounded = round(fps, 2)

        return fps_rounded


class WebcamPublisher(Node):

    def __init__(self):
        super().__init__('webcam_publisher')
       
        # self.publisher_ = self.create_publisher(Image, 'webcam_image', 10)
        self.publisher_ = self.create_publisher(String, 'gesture_char', 10)
        self.timer_ = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Inference node initialized')

        self.bridge = CvBridge()

        self.path = "/home/iyeszin/Documents/hri_sl_ros2/src/ros_sign_language_recognition/ros_sign_language_recognition/weight/cnn_model.h5"

        self.cvFpsCalc = CvFpsCalc(buffer_len=10)
        
        # mediapipe settings
        self.use_static_image_mode = False
        self.min_detection_confidence = 0.7
        self.min_tracking_confidence = 0.7

        self.mp_model, self.sl_classifier = self.load_model()

        self.result = ""

    def timer_callback(self):
        if self.result is not None and self.result != "":
            msg = String()
            msg.data = self.result
            self.publisher_.publish(msg)


    def load_model(self):
        # Model load
        mp_hands = mp.solutions.hands
        hands = mp_hands.Hands(
            static_image_mode=self.use_static_image_mode,
            max_num_hands=1,
            min_detection_confidence=self.min_detection_confidence,
            min_tracking_confidence=self.min_tracking_confidence,
        )

        sl_classifier = models.load_model(self.path)

        return hands, sl_classifier

    # def publish_frame(self, frame):
    #     image_message = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
    #     self.publisher_.publish(image_message)

    def preprocess_image(self, x):
        x = x/255 # normalize the data
        x = x.reshape(-1,28,28,1) # convert it into 28 x 28 gray scaled image
        # x = x.reshape(-1,64,64,3)
        
        return x

    def get_bbox_coordinates(self, handLadmark, image_shape):
        all_x, all_y = [], [] # store all x and y points in list
        for hnd in mp.solutions.hands.HandLandmark:
            all_x.append(int(handLadmark.landmark[hnd].x * image_shape[1])) # multiply x by image width
            all_y.append(int(handLadmark.landmark[hnd].y * image_shape[0])) # multiply y by image height

        return min(all_x), min(all_y), max(all_x), max(all_y) # return as (xmin, ymin, xmax, ymax)
    
    def draw_bounding_rect(self, image, a, b, c, d):
        # bounding rectangle
        cv.rectangle(image, (a, b), (c, d),
                     (0, 255, 0), 2)
        return image
    
    def preprocess_frame(self, frame):
        frame = cv.pyrDown(frame)
        gray = cv.cvtColor(frame, cv.COLOR_RGB2GRAY)
        gray = cv.medianBlur(gray, 3)


        kernel = np.ones((2,2),np.uint8)
        # Perform erosion to remove small objects
        img_erosion = cv.erode(gray, kernel, iterations=1)

        # Perform dilation to fill in gaps
        img_dilation = cv.dilate(img_erosion, kernel, iterations=1)

        resized_image= cv.resize(img_dilation, [28, 28], interpolation=cv.INTER_CUBIC)

        return resized_image

    def capture(self):
        cap = cv.VideoCapture(0)

        # Create background subtractor
        back_sub = cv.createBackgroundSubtractorMOG2(history=600, varThreshold=25, detectShadows=False)

        while True:
            ret, frame = cap.read()

            # Apply background subtraction
            fg_mask = back_sub.apply(frame)

            # Apply binary mask to original image
            masked_frame = cv.bitwise_and(frame, frame, mask=fg_mask)

            # Display masked frame
            if masked_frame is not None and masked_frame.shape[0] > 0 and masked_frame.shape[1] > 0:
                cv.imshow('Masked Frame', masked_frame)

            # Exit on 'q' keypress
            if cv.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv.destroyAllWindows()


    def capture_frames(self):
        cap = cv.VideoCapture(0)
        frames_buffer = []
        back_sub = cv.createBackgroundSubtractorMOG2()

        while True:
            display_fps = self.cvFpsCalc.get()

            # camera capture #####################################################
            ret, image = cap.read()
            if not ret:
                break
            image = cv.flip(image, 1)  # mirror display
            image_height, image_width, _= image.shape # 600, 800, 3

            debug_image = copy.deepcopy(image)
            pred_image = copy.deepcopy(image)

            # Apply background subtraction
            fg_mask = back_sub.apply(pred_image)

            # Apply binary mask to original image
            masked_frame = cv.bitwise_and(pred_image, pred_image, mask=fg_mask)

            my_images_preds = []
            prediction = ""

            
            aspect_ratio = image_height/image_width

            # Conduct detection #############################################################
            image = cv.cvtColor(image, cv.COLOR_BGR2RGB)
            results = self.mp_model.process(image)

            # draw ################################################################
            if results.multi_hand_landmarks is not None:
                for hand_landmarks, handedness in zip(results.multi_hand_landmarks,
                                                    results.multi_handedness):
                    # # palm center of gravity calculation
                    # cx, cy = calc_palm_moment(debug_image, hand_landmarks)
                    # # calculate bounding rectangle
                    # brect = calc_bounding_rect(debug_image, hand_landmarks)
                    # # draw
                    # debug_image = draw_landmarks(debug_image, cx, cy,
                    #                             hand_landmarks, handedness)
                    # debug_image = draw_bounding_rect(use_brect, debug_image, brect)

                    xmin, ymin, xmax, ymax = self.get_bbox_coordinates(hand_landmarks, (image_height, image_width))
                    
                    xmin -= int(20 * (1 + aspect_ratio))
                    ymin -= int(20 * (1 + aspect_ratio))
                    xmax += int(30 * (1 + aspect_ratio))
                    ymax += int(30 * (1 + aspect_ratio))

                    debug_image = self.draw_bounding_rect(debug_image, xmin, ymin, xmax, ymax)
                   

                    cropped_image = pred_image[ymin:ymax, xmin:xmax]
                    
                    if cropped_image is not None and cropped_image.shape[0] > 0 and cropped_image.shape[1] > 0:
                        resized_image = self.preprocess_frame(cropped_image)
                        cv.imshow("crop", resized_image)

                        # resized_image= cv.resize(img_dilation, [64, 64], interpolation=cv.INTER_CUBIC)
                        if handedness.classification[0].label == "Right":
                            resized_image = cv.flip(resized_image, 1)
                        
                        frames_buffer.append(resized_image)

                        if len(frames_buffer) > 30:
                            frames_buffer.pop(0)

                        average_frame = np.mean(frames_buffer, axis=0)
                        
                        my_images_preds = self.preprocess_image(average_frame)
                        my_images_preds = self.sl_classifier.predict(my_images_preds)
                        if np.max(my_images_preds) > 0.60:
                            y_pred_classes = np.argmax(my_images_preds, axis=1)
                            y_pred_probabilities = np.max(my_images_preds, axis=1)
                            # print(f"Image: Predicted class: {y_pred_classes}, Probability: {y_pred_probabilities}")

                            label = np.argmax(my_images_preds)
                            prediction = class_names[label]
                            # result = prediction
                            self.result = prediction
                        else:
                            prediction = ""
                            # result = prediction
                            self.result = prediction
                        
                        # # Detect the gesture and get the timestamp when it's completed
                        # gesture_completed_timestamp = time.time()
                        # print(gesture_completed_timestamp)

                        cv.putText(debug_image, "Prediction:" + str(prediction), (10, 60),
                    cv.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2, cv.LINE_AA)
                    else:
                        # print("None")
                        prediction = "Not detected"
                        cv.putText(debug_image, "Prediction:" + str(prediction), (10, 60),
                    cv.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2, cv.LINE_AA)

            cv.putText(debug_image, "FPS:" + str(display_fps), (10, 30),
                    cv.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2, cv.LINE_AA)
            

            # Key processing (ESC: end) #################################################
            key = cv.waitKey(1)
            if key == 27:  # ESC
                break

            # show image #############################################################
            cv.imshow('SL Detection Demo', debug_image)

        cap.release()
        cv.destroyAllWindows()

    def destroy(self):
        self.node.destroy_node()

class MyClient:
    def __init__(self):
        self.node = rclpy.create_node('my_client')
        self.client = self.node.create_client(CustomService, 'custom_service')
        self.response_received = False

    def send_request(self, result):
        while True:
            # Wait for the service to become available
            self.node.get_logger().info('Waiting for service...')
            self.client.wait_for_service(timeout_sec=1.0)

            # Create the request and set the result
            request = CustomService.Request()
            request.request_data = result

            # Call the service
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)

            # Process the response
            if future.result() is not None:
                response = future.result()
                self.node.get_logger().info('Received response: %s' % response.response_data)
                self.response_received = True
            else:
                self.node.get_logger().info('Service call failed')

            # Break the loop if response received, else continue to wait
            if self.response_received:
                break

            # # Sleep for some time before sending the next request
            # time.sleep(1)

        self.response_received = False  # Reset the flag to receive new requests

    def destroy(self):
        self.node.destroy_node()

def main(args=None):
    rclpy.init(args=args)

    # Create and start the webcam publisher thread
    webcam_publisher = WebcamPublisher()
    webcam_thread = threading.Thread(target=webcam_publisher.capture_frames)
    webcam_thread.start()

    try:
        while rclpy.ok():  # Use rclpy.ok() to check if the ROS context is still alive
            # call the publisher
            webcam_publisher.timer_callback()

            # Add a sleep to avoid a busy-wait loop
            time.sleep(0.1)  # Adjust the sleep duration as needed

    except KeyboardInterrupt:
        pass  # Allow the program to be interrupted with Ctrl+C

    finally:
        # Cleanup
        webcam_publisher.destroy_node()
        webcam_thread.join()
        MyClient.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()