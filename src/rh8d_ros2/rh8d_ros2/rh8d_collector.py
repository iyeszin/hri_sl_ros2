#!usr/bin/env python
import rclpy
from rclpy.node import Node
from rh8d_msgs.msg import AllSensors, AllJoints
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import time
import pandas as pd

class RH8DCollectorNode(Node):
    def __init__(self):
        self.name = "rh8d_collector"
        super().__init__(self.name + "_node")

        # Load parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('hand_polarity', 'L_')
            ]
        )

        try:
            self.polarity = self.get_parameter('hand_polarity').value
            self.get_logger().info("Hand polarity is : %s" % self.polarity)
        except KeyError:
            self.get_logger().info("Could not find hand polarity parameter, setting to default (L_)")
            pass

        # initialize variables
        self.start_time = time.time_ns()/1000000
        self.sensor_dict_list = []
        self.joint_dict_list = []

        # path to data repo
        self.csv_path = "/Documents/hri_sl_ros2/src/rh8d_ros2/collector_data"

        self.callback_group = MutuallyExclusiveCallbackGroup()

        # initialize subscribers
        self.sensor_sub = self.create_subscription(AllSensors, self.polarity + 'AllSensors', self.sensorCallback, 10, callback_group=self.callback_group)
        self.sensor_sub

        self.joint_sub = self.create_subscription(AllJoints, self.polarity + 'Joints', self.jointCallback, 10, callback_group=self.callback_group)
        self.joint_sub

    def jointCallback(self, msg):
        joints = msg.joints
        temp_dict = {}
        for idx, j in enumerate(joints):
            temp_dict['time_ms'] = time.time_ns()/1000000 - self.start_time
            temp_dict['name_' + str(idx)] = j.name
            temp_dict['current_pos_' + str(idx)] = j.present_position
            temp_dict['current_speed_' + str(idx)] = j.present_speed

        self.joint_dict_list.append(temp_dict)

    def sensorCallback(self, msg):
        data = msg.data
        temp_dict = {}
        for i in data:
            temp_dict['time_ms'] = time.time_ns()/1000000 - self.start_time
            temp_dict['fx_'+str(i.id)] = i.fx
            temp_dict['fy_'+str(i.id)] = i.fy
            temp_dict['fz_'+str(i.id)] = i.fz
            temp_dict['abs_'+str(i.id)] = i.abs
            temp_dict['yaw_'+str(i.id)] = i.yaw
            temp_dict['pitch_'+str(i.id)] = i.pitch

        self.sensor_dict_list.append(temp_dict)

    def sensor_to_df(self, data):
        pass


def main(args=None):
    rclpy.init(args=args)

    collector_node = RH8DCollectorNode()

    try:
        rclpy.spin(collector_node)

    finally:
        sensor_df = pd.DataFrame.from_dict(collector_node.sensor_dict_list)
        joint_df = pd.DataFrame.from_dict(collector_node.joint_dict_list)

        sensor_df.to_csv(collector_node.csv_path + "/sensor_data.csv")
        joint_df.to_csv(collector_node.csv_path + "/joint_data.csv")

        collector_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
