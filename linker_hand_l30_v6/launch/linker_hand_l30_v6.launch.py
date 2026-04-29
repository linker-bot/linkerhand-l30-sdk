#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='linker_hand_l30_v6',
            executable='linker_hand_l30_v6',
            name='linker_hand_l30_v6',
            output='screen',
            parameters=[{
                'hand_type': 'left', # 配置Linker Hand灵巧手类型 left | right 字母为小写
                'hand_joint': "L30", # L30 字母为大写
                'canfd_id': 0, # CANFD ID，默认0，CANFD盒子id编号，一般第一个插入的盒子id为0，第二个插入的盒子id为1，依此类推
                'is_touch': True, # 配置Linker Hand灵巧手是否有压力传感器 True | False
            }],
        ),
    ])