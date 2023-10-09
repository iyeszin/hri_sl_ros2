from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    collector_node = Node(package='ros_sign_language_recognition',
                          executable='inference_node',
                          name='inference',
                          output='screen'
                          )

    ld.add_action(collector_node)

    return ld
