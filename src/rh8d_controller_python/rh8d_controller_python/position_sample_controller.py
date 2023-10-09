#!usr/bin/env python
import time

import rclpy
from rclpy.node import Node
from rh8d_msgs.msg import JointSetSpeedPos, JointListSetSpeedPos
import math

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

        self.timer_period = 1 / self.frequency
        self.main_loop_sub = self.create_timer(self.timer_period, self.mainLoopCallback)

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

    def mainLoopCallback(self):

        sine = math.sin(self.timer_period/8 * self.t)
        self.target_position_list[4] = int(3000 * sine)
        self.target_position_list[5] = int(3000 * sine)
        self.target_position_list[6] = int(3000 * sine)
        self.target_position_list[7] = int(3000 * sine)


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
