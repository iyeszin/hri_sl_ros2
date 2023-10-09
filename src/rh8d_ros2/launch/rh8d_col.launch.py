#!/usr/bin/env python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    sensor_param_path = os.path.join(get_package_share_directory('rh8d_hw'), 'config', 'sensors_left.yaml')

    collector_node = Node(package='rh8d_ros2',
                          executable='rh8d_collector_node',
                          name='rh8d_collector',
                          parameters=[sensor_param_path],
                          output='screen'
                          )

    ld.add_action(collector_node)

    return ld
