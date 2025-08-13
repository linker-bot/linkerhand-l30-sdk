#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='linker_hand_l30',
            executable='linker_hand_l30',
            name='linker_hand_l30',
            output='screen',
            parameters=[{
                'sdk_versoin': '2.0.1',
                'hand_type': 'right',
                'hand_joint': "L30",
                'is_touch': False,
                #'can': 'can0', # 这里需要修改为实际的CAN总线名称 如果是win系统则 PCAN_USBBUS1
            }],
        ),
    ])