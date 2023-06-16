from launch import LaunchDescription
from launch_ros.actions import Node

import launch.actions
import launch_ros.actions

def generate_launch_description():

    ros2_control_node = Node(
            package='charmie_bot',
            executable='control_test.py',
            output='screen',
            arguments=["0.14"]
         )

    return LaunchDescription([
        ros2_control_node
    ])