#!/usr/bin/env python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    param_path = os.path.join(get_package_share_directory('rh8d_hw'), 'config', 'RH8D_tactile_L.yaml')

    hw_node = Node(package='rh8d_hw',
                   executable='rh8d_tactile_combined',
                   name='rh8d_tactile',
                   parameters=[param_path],
                   output='screen'
                   )

    ld.add_action(hw_node)

    return ld
