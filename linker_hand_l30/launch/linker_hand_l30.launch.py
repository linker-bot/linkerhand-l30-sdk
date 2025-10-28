#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='linker_hand_l30',
            executable='linker_hand_l30_node',
            name='linker_hand_l30_node',
            output='screen',
            parameters=[{
                'usb_port': '/dev/ttyUSB0',
                'hand_type': 'left',
                'hand_joint': "L30",
                'is_touch': True,
            }],
        ),
    ])