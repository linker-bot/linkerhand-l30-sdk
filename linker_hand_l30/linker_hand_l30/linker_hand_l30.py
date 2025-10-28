#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy,time, threading, json, sys
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from .utils.color_msg import ColorMsg
from l30_hand_api import L30HandAPI
from l30_hand_api.motor_mapping import get_table_columns, get_motor_id, get_joint_info


class LinkerHandL30(Node):
    def __init__(self):
        super().__init__('linker_hand_l30_node')
        # 声明参数（带默认值）
        self.declare_parameter('usb_port', '/dev/ttyUSB0')
        self.declare_parameter('hand_type', 'right')
        self.declare_parameter('hand_joint', 'L30')
        self.declare_parameter('is_touch', False)
        
        # 获取参数值
        self.usb_port = self.get_parameter("usb_port").value
        self.hand_type = self.get_parameter('hand_type').value
        self.hand_joint = self.get_parameter('hand_joint').value
        self.is_touch = self.get_parameter('is_touch').value
        self.hand_setting_sub = self.create_subscription(String,'/cb_hand_setting_cmd', self.hand_setting_cb, 10)
        self.hand_api = L30HandAPI()
        self.last_pose = []
        self.lock = True
        success, msg, hand_type = self.hand_api.connect_motor(self.usb_port) # "/dev/ttyUSB0"
        if success:
            # 确保界面模块使用正确的HAND_TYPE
            from l30_hand_api.motor_mapping import set_and_get_hand_type_by_wrist_id
            if 117 in self.hand_api.motors:
                set_and_get_hand_type_by_wrist_id(117)
                print("检测到左手，设置HAND_TYPE为left")
            elif 17 in self.hand_api.motors:
                set_and_get_hand_type_by_wrist_id(17)
                print("检测到右手，设置HAND_TYPE为right")
            
            self.is_connected = True
        self.publisher_hand_state_ = self.create_publisher(JointState, f'/cb_{self.hand_type}_hand_state', 10)
        if success:
            ColorMsg(msg="连接手部设备成功", color="green")
        else:
            ports = self.hand_api.get_available_ports()
            ColorMsg(msg=f"连接手部设备失败: {msg},可能当前端口没有权限", color="red")
            print("==========当前可用端口============", flush=True)
            print(ports, flush=True)
            print("================================", flush=True)
            return
        self.control_topci = f'/cb_{self.hand_type}_hand_control_cmd'
        self.subscription = self.create_subscription(
            JointState,
            self.control_topci,
            self.hand_callback,
            10)
        # 设置运动速度（0-1000）
        
        # 获取所有关节名称和对应的电机ID
        self.joint_order = get_table_columns()[1:] # 跳过序号列
        self.joint_names = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7', 'joint8', 'joint9', 'joint10', 'joint11', 'joint12', 'joint13', 'joint14', 'joint15', 'joint16']
        ColorMsg(msg=f"关节名称: {self.joint_order}", color="green")
        self.motor_ids = [get_motor_id(joint_name) for joint_name in self.joint_order]
        ColorMsg(msg=f"电机ID: {self.motor_ids}", color="green")
        self.hand_api.set_all_velocity_sync(200)
        self.hand_api.open_hand()
        time.sleep(1)  # 等待手部设备准备就绪
        self.pub_thread = threading.Thread(target=self.pub_hand_state)
        self.pub_thread.daemon = True  # 设置为守护线程
        self.pub_thread.start()
        
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        

    def pub_hand_state(self):
        while rclpy.ok():
            msg = self.joint_msg(positions=self.last_pose, joint_names=self.joint_names)
            self.publisher_hand_state_.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)  # 确保节点在发布消息时不会阻塞
            time.sleep(0.02)  # 控制发布频率


    def timer_callback(self):
        
        '''
        ['大拇指侧摆', '拇指旋转', '拇指弯曲', '拇指指尖', '食指侧摆', '食指指根弯曲', '食指指尖', '中指侧摆', '中指指根', '中指指尖', '无名指侧摆', '无名指指根', '无名指指尖', '小指侧摆', '小指指根', '小指指尖', '手腕']
        [3, 4, 1, 2, 14, 15, 16, 13, 8, 9, 5, 7, 6, 12, 10, 11, 17]
        '''
        if len(self.get_publishers_info_by_topic(self.control_topci)) > 0 and len(self.last_pose) > 0:
            #print(f"当前state: {self.last_pose}", flush=True)
            pass
        else:
            current_pose = self.hand_api.read_current_positions()
            for key,value in current_pose.items():
                if key not in self.motor_ids:
                    print(f"警告：电机 {key} 未在 motor_ids 中找到，跳过", flush=True)
                    continue
                # 将电机角度转换为关节角度
                current_pose[key] = self.motor_to_joint_angle(key, value)
            tmp_list = []
            tmp_list = [int(current_pose.get(key, 0.0)) for key in self.motor_ids]
            self.last_pose = tmp_list
            #print(f"获取state: {self.last_pose}", flush=True)

    def hand_callback(self, msg):
        positions = list(msg.position)
        if not positions:
            return
        result_dict = dict(zip(self.motor_ids, positions))
        # 将关节角度转换为电机角度
        result_dict = {
            key: self.joint_to_motor_angle(key, value)
            for key, value in result_dict.items()
        }
        # 同步设置电机位置
        res = self.hand_api.set_motors_position_sync(result_dict)
        # 保存上一次的姿态（确保 positions 已定义）
        if 'positions' in locals():
            self.last_pose = positions
        else:
            self.last_pose = []



    def joint_to_motor_angle(self, motor_id: int, joint_angle: float) -> float:
        """
        将关节角度转换为电机角度

        Args:
            motor_id: 电机ID
            joint_angle: 关节角度（UI显示值）

        Returns:
            float: 电机角度值
        """
        # 获取关节信息
        joint_info = get_joint_info(motor_id)
        if not joint_info:
            print(f"警告：电机 {motor_id} 未找到关节信息，使用原始角度")
            return joint_angle

        # 获取关节范围
        min_angle, max_angle = joint_info['range']

        # 限制关节角度在有效范围内
        clamped_angle = max(min_angle, min(max_angle, joint_angle))

        # 根据方向调整角度
        if joint_info.get('direction', 1) < 0:
            # 反向映射
            motor_angle = 180 - clamped_angle
        else:
            motor_angle = clamped_angle + 180
        return motor_angle

    def motor_to_joint_angle(self, motor_id: int, motor_angle: float) -> float:
        """
        将电机角度转换为关节角度

        Args:
            motor_id: 电机ID
            motor_angle: 电机角度值

        Returns:
            float: 关节角度（UI显示值）
        """
        # 获取关节信息
        joint_info = get_joint_info(motor_id)
        if not joint_info:
            print(f"警告：电机 {motor_id} 未找到关节信息，使用原始角度")
            return motor_angle

        # 获取关节范围
        min_angle, max_angle = joint_info['range']

        # 根据方向调整角度
        if joint_info.get('direction', 1) < 0:
            # 反向映射
            joint_angle = 180 - motor_angle
        else:
            joint_angle = motor_angle - 180

        # 限制关节角度在有效范围内
        joint_angle = max(min_angle, min(max_angle, joint_angle))

        #print(f"电机 {motor_id} - 电机角度 {motor_angle:.1f}° 转换为关节角度 {joint_angle:.1f}°")
        return joint_angle    

    def joint_msg(self,positions=[],joint_names=[]):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = joint_names
        joint_state.position = positions
        joint_state.velocity = [0.0] * len(positions)
        joint_state.effort = [0.0] * len(positions)
        return joint_state
    
    def disconnect_motor(self):
        """断开电机连接并更新界面状态"""
        print("正在断开电机连接...", flush=True)
        if self.is_connected:
            self.hand_api.disconnect_motor()

        # 重置连接状态
        self.is_connected = False

    def hand_setting_cb(self,msg):
        '''控制命令回调'''
        data = json.loads(msg.data)
        print(f"Received setting command: {data['setting_cmd']}",flush=True)
        try:
            if data["setting_cmd"] == "set_speed": # Set speed
                speed = int(data["params"]["speed"])
                if speed < 0 or speed > 1000:
                    print("Speed must be between 0 and 1000", flush=True)
                    return
                self.hand_api.set_all_velocity_sync(speed)
                ColorMsg(msg=f"Setting Speed: {speed}", color="green")
        except Exception as e:
            print(f"Error in hand_setting_cb: {e}", flush=True)
            ColorMsg(msg=f"Error in hand_setting_cb: {e}", color="red")

def main(args=None):
    rclpy.init(args=args)
    node = LinkerHandL30()
    try:
        rclpy.spin(node)         # 主循环，监听 ROS 回调
    except KeyboardInterrupt:
        print("收到 Ctrl+C，准备退出...")
    finally:      # 关闭 CAN 或其他硬件资源
        node.disconnect_motor()
        node.destroy_node()      # 销毁 ROS 节点
        rclpy.shutdown()         # 关闭 ROS
        print("程序已退出。")