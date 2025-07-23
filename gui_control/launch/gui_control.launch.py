#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gui_control',
            executable='gui_control',
            name='left_hand_control_node',
            output='screen',
            parameters=[{
                'hand_type': 'left',
                'hand_joint': "L30",
                'topic_hz': 30,
                'is_touch': True,
            }],
        ),
    ])
