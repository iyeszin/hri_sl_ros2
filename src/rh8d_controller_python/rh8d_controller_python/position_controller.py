#!usr/bin/env python
import time
import math
import csv

import rclpy
from rclpy.node import Node
from rh8d_msgs.msg import JointSetSpeedPos, JointListSetSpeedPos
from rh8d_msgs.srv import CustomService

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

        # server node
        self.srv = self.create_service(
            CustomService,
            'custom_service',
            self.callback_function
        )

        self.saved_data = None

    def callback_function(self, request, response):
        # self.get_logger().info('Received request: %s' % request.request_data)

        # # Process the request and generate a response
        # response.response_data = request.request_data
        # response.success = True

        # return response


        self.get_logger().info('Received request: %s' % request.request_data)

        # # Save the request data
        # self.saved_data = request.request_data
        # print(self.saved_data)

        # Call another function and pass the request data
        self.mainLoopCallback(request.request_data)

        # Process the request and generate a response
        response.success = True
        response.response_data = "Done"
        # response.response_data = request.request_data
        return response
    
    def read_csv_data(self, file_path, sign):
        target_position_list = []

        with open(file_path, 'r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                if sign == row['character']:
                    target_position_list = [int(row['w_rotation']), int(row['w_flexion']), int(row['w_adduction']),
                                            int(row['th_adduction']), int(row['th_flexion']), int(row['ix_flexion']),
                                            int(row['middle_flexion']), int(row['ring_ltl_flexion'])]
                    break

        return target_position_list

    def mainLoopCallback(self, sign):

        print(sign)

        if sign.strip() == "":
            # If the request is blank, do not change the target position
            pass
        else:
            # If the request is not blank, update the target position
            self.target_position_list = self.read_csv_data("/home/iyeszin/Documents/hri_sl_ros2/position_trajectories.csv", sign)

        # Rest of your code remains unchanged
        for name, position, speed, joint in zip(self.joint_names, self.target_position_list, self.target_speed_list, self.lone_joints):
            joint.name = name
            joint.target_pos = position
            joint.target_speed = speed

        self.msg.joints = self.lone_joints

        self.joint_pub.publish(self.msg)

        if self.t < 200:
            self.t += 1
        else:
            self.t = 0

        # # Check if the request is blank
        # if sign.strip() == "":
        #     # Send completion message if the last character is processed
        #     self.get_logger().info("Hand has finished executing all requested actions.")



    def initPosition(self):
        self.target_position_list = [2048, 2048, 2048, 0, 0, 0, 0, 0]
        self.target_speed_list = [0, 0, 0, 0, 0, 0, 0, 0]

        for name, position, speed, joint in zip(self.joint_names, self.target_position_list, self.target_speed_list, self.lone_joints):
            joint.name = name
            joint.target_pos = position
            joint.target_speed = speed

        self.msg.joints = self.lone_joints

        self.joint_pub.publish(self.msg)



def main(args=None):
    rclpy.init(args=args)

    controller_node = RH8DSampleController()

    try:
        rclpy.spin(controller_node)

    finally:
        controller_node.initPosition()
        time.sleep(2)
        controller_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
