#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
L30 hand API

API for CANFD protocol control
"""
import time,os,sys
from typing import  List, Optional, Tuple, Dict, Any
import numpy as np
from .core.linker_hand_l30_v6_canfd import (
    L30DexterousHandController,
    L30CANFDProtocol,
    Command,
    HandType,
    JOINT_NAME_EN,
    JOINT_NAME_CN,
    StatusCode,
)
from .utils.color_msg import ColorMsg

class LinkerHandL30API:
    def __init__(self, hand_joint="L30", hand_type="right", device_id=0x06,canfd_id=0) -> None:
        """
        Args:
            hand_joint: 手关节类型，默认L30
            hand_type: 手类型，默认right
            device_id: 设备ID，默认0x06, 灵巧手ID
            canfd_id: CANFD ID，默认0，CANFD盒子id编号，一般第一个插入的盒子id为0，第二个插入的盒子id为1，依此类推
        """
        
        self.hand_joint = hand_joint
        self.hand_type = hand_type
        self.hand_controller = L30DexterousHandController(device_id=device_id,canfd_id=canfd_id)
        ColorMsg(msg=f"开始连接L30...", color="yellow")
        con = self.hand_controller.connect()
        time.sleep(0.001)
        if con == False:
            ColorMsg(msg=f"Linker Hand L30 连接失败，CANFD连接失败", color="red")
        elif self.hand_type == self.hand_controller.hand_type:
            ColorMsg(msg=f"连接成功,Linker Hand {self.hand_joint} - {self.hand_type}在CANFD编号为{canfd_id}设备上", color="green")
        else:
            ColorMsg(msg=f"Linker Hand L30 连接失败，hand_type不匹配", color="red")
            raise ValueError("Hand type is incorrect")
        self.joint_name_en = JOINT_NAME_EN
        self.joint_name_cn = JOINT_NAME_CN
        

    def set_joint_torques(self, torques: List[int] = [1024] * 17)->None:
        """
        设置17个关节扭矩，默认1024
        
        Args:
            torques: 17个关节扭矩值，范围-2047~2047，单位6.5mA
        """
        if not isinstance(torques, list) or len(torques) != 17:
            raise ValueError(f"需要17个长度的list，当前为{len(torques) if isinstance(torques, list) else type(torques)}")
        
        tor = []
        for i, val in enumerate(torques):
            if not isinstance(val, int):
                raise TypeError(f"第{i}个值必须是int类型，当前为{type(val)}: {val}")
            tor.append(max(-2047, min(2047, val)))
        #print(f"设置扭矩:{tor}")
        self.hand_controller.set_torques(tor)

    def set_velocities(self, velocities: List[int] = [150] * 17)->None:
        """
        设置17个关节速度,默认150
        
        Args:
            velocities: 17个关节速度值，范围0~150，单位0.732RPM
        """
        if not isinstance(velocities, list) or len(velocities) != 17:
            raise ValueError(f"需要17个长度的list，当前为{len(velocities) if isinstance(velocities, list) else type(velocities)}")
        
        vel = []
        for i, val in enumerate(velocities):
            if not isinstance(val, int):
                raise TypeError(f"第{i}个值必须是int类型，当前为{type(val)}: {val}")
            vel.append(max(0, min(150, val)))
        #print(f"设置速度:{vel}")
        self.hand_controller.set_velocities(vel)

    def set_positions(self, positions: List[int] = [0] * 17)->None:
        """
        设置17个关节位置,默认0
        
        Args:
            positions: 17个关节位置值，范围-2047~2047，单位0.732RPM
        """
        # positions的数据已经在L30DexterousHandController类中进行验证
        self.hand_controller.set_positions(positions)


    def get_all_state(self,is_touch=False)->Dict[str, Any]:
        ''' 获取所有状态 
        
        Returns:
            Dict
            all_state: 所有状态，包括：
                - positions: 关节位置
                - velocities: 关节速度
                - temperatures: 关节温度
                - currents: 关节电流
                - error_codes: 错误码
                - matrix_touch: 五指指尖压感矩阵
                
        '''
        all_state = self.hand_controller.get_all_state(is_touch=is_touch)
        if all_state is None:
            raise ValueError("获取所有状态失败")
        return all_state

    def get_joint_state(self)->Optional[List[int]]:
        """
        获取17个关节状态
        
        Returns:
            List[int]
            17个关节状态
        """
        joint_state = self.hand_controller.protocol.get_joint_positions()
        if joint_state is None:
            raise ValueError("获取关节状态失败")
        return joint_state

    def get_joint_velocities(self)->Optional[List[int]]:
        """
        获取17个关节速度
        
        Returns:
            List[int]
            17个关节速度
        """
        joint_velocities = self.hand_controller.protocol.get_joint_velocities()
        if joint_velocities is None:
            raise ValueError("获取关节速度失败")
        return joint_velocities

    def get_joint_torques(self)->Optional[List[int]]:
        """
        获取17个关节扭矩
        
        Returns:
            List[int]
            17个关节扭矩
        """
        joint_torques = self.hand_controller.protocol.get_joint_torques()
        if joint_torques is None:
            raise ValueError("获取关节扭矩失败")
        return joint_torques

    def get_joint_temperatures(self)->Optional[List[int]]:
        """
        获取17个关节温度
        
        Returns:
            List[int]
            17个关节温度
        """
        joint_temperatures = self.hand_controller.protocol.get_joint_temperatures()
        if joint_temperatures is None:
            raise ValueError("获取关节温度失败")
        return joint_temperatures

    def get_joint_currents(self)->Optional[List[int]]:
        """
        获取17个关节电流
        
        Returns:
            List[int]
            17个关节电流
        """
        joint_currents = self.hand_controller.protocol.get_joint_currents()
        if joint_currents is None:
            raise ValueError("获取关节电流失败")

    def get_matrix_touch(self)->Dict[str, List[int]]:
        """
        获取五指指尖压感矩阵
        
        Returns:
            List[int]
            五指指尖压感矩阵
        """
        matrix_touch = self.hand_controller.get_matrix_touch()
        if matrix_touch is None:
            raise ValueError("获取五指指尖压感矩阵失败")
        return matrix_touch

    def disconnect(self):
        self.hand_controller.disconnect()

    def __del__(self) -> None:
        self.hand_controller.disconnect()
        time.sleep(0.001)

if __name__ == "__main__":
    api = LinkerHandL30API(hand_joint="L30", hand_type="right", device_id=0x06)
    api.set_positions(positions=[1000, 2, 145, -13, 3, 2, -3, 7, 3, 5, 12, -3, 1, 1, 11, 9, 137])

    