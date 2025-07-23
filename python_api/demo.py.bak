#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
L30_APP - 
主要功能包括：
- 串口连接和断开
- 电机使能控制
- 单个/多个电机位置控制
- 速度控制
- 动作序列的保存和加载
- 预设手势动作
- 手部校准
"""

import os
import sys
import time
import platform
import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
from tkinter import simpledialog
import threading
import serial.tools
import serial.tools.list_ports
from ttkbootstrap import Style  # 使用ttkbootstrap美化界面
import csv
from tkinter import filedialog

# 从安装的包中导入API和必要的函数
from l30_hand_api import L30HandAPI
from l30_hand_api.motor_mapping import get_table_columns, get_motor_id, get_joint_info

# Dynamixel电机控制表地址定义
ADDR_OPERATING_MODE = 11      # 操作模式地址
ADDR_TORQUE_ENABLE = 64      # 扭矩使能地址
ADDR_GOAL_POSITION = 116     # 目标位置地址
ADDR_PRESENT_POSITION = 132  # 当前位置地址
ADDR_MOVING = 122           # 运动状态地址
ADDR_PROFILE_VELOCITY = 112  # 速度控制地址

# 协议版本和通信参数设置
PROTOCOL_VERSION = 2.0       # Dynamixel协议2.0版本
BAUDRATE = 1000000          # 串口波特率1M
DEFAULT_VELOCITY = 255      # 默认速度（最大值）
MAX_VELOCITY = 1000         # 最大速度值
DEFAULT_INTERVAL = 0.2     # 默认动作间隔时间（秒）

# 电机控制参数
TORQUE_ENABLE = 1          # 扭矩使能
TORQUE_DISABLE = 0         # 扭矩禁用
DXL_MINIMUM_POSITION_VALUE = 0     # 最小位置值
DXL_MAXIMUM_POSITION_VALUE = 4095  # 最大位置值（12位分辨率）
POSITION_CONTROL_MODE = 3          # 位置控制模式

# 系统相关配置
SYSTEM = platform.system()
if SYSTEM == 'Windows':
    DEFAULT_FONT = ('Microsoft YaHei', 10)
    CHINESE_FONT = ('Microsoft YaHei', 10)
    TITLE_FONT = ('Microsoft YaHei', 11, 'bold')
    BUTTON_FONT = ('Microsoft YaHei', 10)
    TABLE_FONT = ('Microsoft YaHei', 10)
else:  # Linux
    DEFAULT_FONT = ('DejaVu Sans', 10)
    CHINESE_FONT = ('WenQuanYi Micro Hei', 10)
    TITLE_FONT = ('DejaVu Sans', 11, 'bold')
    BUTTON_FONT = ('DejaVu Sans', 10)
    TABLE_FONT = ('DejaVu Sans', 10)

class MultiMotorControlGUI:
    """
    实现了机械手控制界面的所有功能，包括：
    - 界面布局
    - 用户交互
    - 动作序列管理
    - 数据保存和加载
    """
    
    def __init__(self, root):
        """
        初始化控制界面
        Args:
            root: tkinter主窗口对象
        """
        self.root = root
        self.root.title("L30_demo")
        
        # 初始化API实例
        self.hand_api = L30HandAPI()
        self.hand_api.gui = self  # 设置GUI引用
        
        # 初始化其他属性
        self.is_connected = False
        self.sequence_running = False      # 序列运行状态标志
        self.stop_flag = False
        self.loop_count_label = None      # 循环计数标签
        
        # 电机控制组件存储
        self.motors = {}  # 存储电机控制组件
        # 初始化所有电机的控制组件
        for motor_id in range(1, 18):
            self.motors[motor_id] = {
                'label': None,             # 电机标签
                'slider': None,            # 滑块控件
                'entry': None,             # 输入框控件
                'current_pos': None        # 当前位置标签
            }
        
        # 界面控件
        self.velocity_label = None  # 速度显示标签
        self.velocity_slider = None  # 速度滑块
        self.port_combobox = None   # 串口选择下拉框
        
        # 设置界面布局
        self.setup_gui()

        # 初始化串口列表
        self.refresh_ports()

    def setup_gui(self):
        """设置主界面布局"""
        # 设置全局字体
        self.root.option_add('*Font', DEFAULT_FONT)
        
        # 设置 ttk 样式
        style = ttk.Style()
        
        # 标签样式
        style.configure('TLabel', font=DEFAULT_FONT)
        style.configure('Title.TLabel', font=TITLE_FONT)
        style.configure('Chinese.TLabel', font=CHINESE_FONT)  # 中文标签样式
        
        # 按钮样式
        style.configure('TButton', font=BUTTON_FONT)
        style.configure('Primary.TButton', font=BUTTON_FONT)
        style.configure('Success.TButton', font=BUTTON_FONT)
        style.configure('Info.TButton', font=BUTTON_FONT)
        style.configure('Warning.TButton', font=BUTTON_FONT)
        style.configure('Danger.TButton', font=BUTTON_FONT)
        style.configure('Chinese.TButton', font=CHINESE_FONT)  # 中文按钮样式
        
        # 输入框样式
        style.configure('TEntry', font=DEFAULT_FONT)
        
        # 标签框架样式
        style.configure('TLabelframe.Label', font=CHINESE_FONT)  # 使用中文字体
        
        # 表格样式
        style.configure('Treeview', font=TABLE_FONT)
        style.configure('Treeview.Heading', font=CHINESE_FONT)  # 表格标题使用中文字体
        
        # 滑块样式
        style.configure('TScale', font=DEFAULT_FONT)
        
        # 创建主框架
        self.main_frame = ttk.Frame(self.root, padding="5")
        self.main_frame.pack(fill=tk.BOTH, expand=True)
        
        # 设置各个区域
        self.setup_top_control()
        self.setup_motor_control()
        self.setup_data_table()
        self.setup_bottom_control()
        
        # 初始状态下禁用操作控件
        self.disable_operation_controls()

    def setup_top_control(self):
        """设置顶部控制区域"""
        top_frame = ttk.Frame(self.main_frame)
        top_frame.pack(fill=tk.X, pady=5)
        
        # 串口控制区域
        port_frame = ttk.LabelFrame(top_frame, text="通信设置", padding=5)
        port_frame.pack(side=tk.LEFT, padx=5)
        
        # 串口选择组件
        ttk.Label(port_frame, text="串口:").pack(side=tk.LEFT, padx=5)
        self.port_combobox = ttk.Combobox(port_frame, width=10)
        self.port_combobox.pack(side=tk.LEFT, padx=5)
        
        # 串口控制按钮
        ttk.Button(port_frame, text="刷新", command=self.refresh_ports, 
                  style='primary.TButton').pack(side=tk.LEFT, padx=5)
        self.connect_btn = ttk.Button(port_frame, text="连接", command=self.toggle_connection, style='success.TButton')
        self.connect_btn.pack(side=tk.LEFT, padx=5)

        # 电机使能控制按钮
        self.torque_btn = ttk.Button(port_frame, text="开启电机出力", 
                                   command=self.toggle_all_torque, 
                                   style='warning.TButton')
        self.torque_btn.pack(side=tk.LEFT, padx=5)
        self.torque_btn.state(['disabled'])

        # 预设动作按钮
        self.open_hand_btn = ttk.Button(port_frame, text="五指张开", 
                                      command=self.open_hand, 
                                      style='info.TButton')
        self.open_hand_btn.pack(side=tk.LEFT, padx=5)
        self.open_hand_btn.state(['disabled'])

        # 校准按钮
        self.calibrate_btn = ttk.Button(port_frame, text="手部校准", 
                                      command=self.calibrate_hand, 
                                      style='info.TButton')
        self.calibrate_btn.pack(side=tk.LEFT, padx=5)
        self.calibrate_btn.state(['disabled'])

        # 速度控制区域
        velocity_frame = ttk.LabelFrame(top_frame, text="速度控制", padding=5)
        velocity_frame.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
        
        # 速度控制组件
        ttk.Label(velocity_frame, text="全局速度:").pack(side=tk.LEFT, padx=5)
        
        # 创建并初始化速度显示标签
        self.velocity_label = ttk.Label(velocity_frame, text=str(MAX_VELOCITY))
        
        # 创建速度滑块
        self.velocity_slider = ttk.Scale(velocity_frame, from_=0, to=MAX_VELOCITY, 
                                       orient=tk.HORIZONTAL, 
                                       command=self.on_velocity_change)
        self.velocity_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        self.velocity_slider.set(MAX_VELOCITY)  # 设置初始速度为最大值
        
        # 放置速度显示标签
        self.velocity_label.pack(side=tk.LEFT, padx=5)

    def setup_motor_control(self):
        """设置电机控制区域"""
        control_frame = ttk.LabelFrame(self.main_frame, text="电机控制", padding=5)
        control_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        # Get table columns (excluding index column)
        columns = get_table_columns()[1:]
        
        # Create control components for each joint
        for joint_name in columns:
            motor_id = get_motor_id(joint_name)
            if motor_id is None:
                continue
                
            motor_frame = ttk.Frame(control_frame)
            motor_frame.pack(fill=tk.X, pady=2)
            
            # Get joint information
            joint_info = get_joint_info(motor_id)
            if not joint_info:
                continue
                
            min_angle, max_angle = joint_info['range']
            
            # Joint name label with Chinese font
            label = ttk.Label(motor_frame, text=f"{joint_name}", width=20, style='Chinese.TLabel')
            label.pack(side=tk.LEFT, padx=5)
            
            # Position control slider
            slider = ttk.Scale(motor_frame, from_=min_angle, to=max_angle, orient=tk.HORIZONTAL,
                             command=lambda v, m=motor_id: self.on_slider_change(m, v))
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
            
            # Position input box
            entry = ttk.Entry(motor_frame, width=8)
            entry.pack(side=tk.LEFT, padx=5)
            entry.bind('<Return>', lambda e, m=motor_id: self.on_entry_change(m))
            entry.bind('<FocusOut>', lambda e, m=motor_id: self.on_entry_change(m))
            
            # Current position display label with Chinese font
            current_pos = ttk.Label(motor_frame, text="当前位置: 0.0°", style='Chinese.TLabel')
            current_pos.pack(side=tk.LEFT, padx=5)
            
            # Store references to controls
            self.motors[motor_id] = {
                'label': label,
                'slider': slider,
                'entry': entry,
                'current_pos': current_pos
            }

    def setup_data_table(self):
        """设置动作数据表格区域"""
        table_frame = ttk.LabelFrame(self.main_frame, text="动作数据", padding=5)
        table_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        # 获取表格列名
        self.table_columns = get_table_columns()
        
        # 创建表格控件
        self.tree = ttk.Treeview(table_frame, columns=self.table_columns, show='headings', height=6)
        
        # 设置列标题和宽度，并设置居中对齐
        for col in self.table_columns:
            self.tree.heading(col, text=col, anchor=tk.CENTER)  # 列标题居中
            self.tree.column(col, width=100, anchor=tk.CENTER)   # 列内容居中，增加列宽到100
        
        # 添加垂直和水平滚动条
        y_scrollbar = ttk.Scrollbar(table_frame, orient=tk.VERTICAL, command=self.tree.yview)
        x_scrollbar = ttk.Scrollbar(table_frame, orient=tk.HORIZONTAL, command=self.tree.xview)
        self.tree.configure(yscrollcommand=y_scrollbar.set, xscrollcommand=x_scrollbar.set)
        
        # 使用网格布局放置表格和滚动条
        self.tree.grid(row=0, column=0, sticky='nsew')
        y_scrollbar.grid(row=0, column=1, sticky='ns')
        x_scrollbar.grid(row=1, column=0, sticky='ew')
        
        # 配置网格权重，使表格能够自适应大小
        table_frame.grid_columnconfigure(0, weight=1)
        table_frame.grid_rowconfigure(0, weight=1)

        # 创建编辑框
        style = ttk.Style()
        style.configure('Edit.TEntry', padding='2 2 2 2')  # 添加内边距
        self.edit_entry = ttk.Entry(self.tree, style='Edit.TEntry', justify='center')  # 居中对齐
        
        # 绑定双击事件和编辑框事件
        self.tree.bind('<Double-1>', self.on_double_click)
        self.edit_entry.bind('<Return>', self.finish_edit)  # 回车键完成编辑
        self.edit_entry.bind('<Escape>', self.cancel_edit)  # ESC键取消编辑
        self.edit_entry.bind('<FocusOut>', self.finish_edit)  # 失去焦点时完成编辑

    def setup_bottom_control(self):
        """设置底部控制区域"""
        bottom_frame = ttk.Frame(self.main_frame)
        bottom_frame.pack(fill=tk.X, pady=5)
        
        # 左侧按钮组 - 动作控制
        left_btn_frame = ttk.Frame(bottom_frame)
        left_btn_frame.pack(side=tk.LEFT)
        
        # 添加循环计数显示标签

        
        ttk.Button(left_btn_frame, text="读取当前位置", 
                  command=self.read_current_positions, 
                  style='info.TButton').pack(side=tk.LEFT, padx=2)
        ttk.Button(left_btn_frame, text="保存当前位置", 
                  command=self.save_current_positions, 
                  style='info.TButton').pack(side=tk.LEFT, padx=2)
        ttk.Button(left_btn_frame, text="运行选中行", 
                  command=self.run_selected_row, 
                  style='success.TButton').pack(side=tk.LEFT, padx=2)
        
        # 中间控制组 - 序列运行控制
        mid_frame = ttk.Frame(bottom_frame)
        mid_frame.pack(side=tk.LEFT, padx=20)

        # 时间间隔设置
        ttk.Label(mid_frame, text="间隔时间(秒):").pack(side=tk.LEFT, padx=2)
        self.interval_entry = ttk.Entry(mid_frame, width=5)
        self.interval_entry.insert(0, str(DEFAULT_INTERVAL))
        self.interval_entry.pack(side=tk.LEFT, padx=2)

        # 循环次数设置
        ttk.Label(mid_frame, text="循环次数:").pack(side=tk.LEFT, padx=2)
        self.loop_entry = ttk.Entry(mid_frame, width=5)
        self.loop_entry.insert(0, "1")
        self.loop_entry.pack(side=tk.LEFT, padx=2)
        
        # 运行序列按钮
        self.run_all_btn = ttk.Button(mid_frame, text="运行所有序列", 
                                    command=lambda: self.run_all_sequences(
                                        float(self.interval_entry.get()),
                                        int(self.loop_entry.get()),
                                        self.tree.get_children()
                                    ), 
                                    style='success.TButton')
        self.run_all_btn.pack(side=tk.LEFT, padx=2)
        
        # 停止序列按钮
        self.stop_btn = ttk.Button(mid_frame, text="停止运行", 
                                 command=self.stop_sequences, 
                                 style='danger.TButton')
        self.stop_btn.pack(side=tk.LEFT, padx=2)
        self.stop_btn.state(['disabled'])  # 初始状态禁用

        self.loop_count_label = ttk.Label(mid_frame, text="循环次数: 0/当前循环: 0")
        self.loop_count_label.pack(side=tk.LEFT, padx=5)
        # 右侧按钮组 - 数据管理
        right_btn_frame = ttk.Frame(bottom_frame)
        right_btn_frame.pack(side=tk.RIGHT)
        
        ttk.Button(right_btn_frame, text="保存动作数据", 
                  command=self.save_sequence_data, 
                  style='primary.TButton').pack(side=tk.LEFT, padx=2)
        ttk.Button(right_btn_frame, text="读取动作数据", 
                  command=self.load_sequence_data, 
                  style='primary.TButton').pack(side=tk.LEFT, padx=2)
        ttk.Button(right_btn_frame, text="删除选中行", 
                  command=self.delete_selected_row, 
                  style='danger.TButton').pack(side=tk.LEFT, padx=2)
        ttk.Button(right_btn_frame, text="清空表格", 
                  command=self.clear_table, 
                  style='danger.TButton').pack(side=tk.LEFT, padx=2)

    def on_double_click(self, event):
        """处理表格双击事件，显示编辑框"""
        # 获取点击的区域
        region = self.tree.identify_region(event.x, event.y)
        if region != "cell":
            return
            
        # 获取点击的单元格信息
        column = self.tree.identify_column(event.x)
        item = self.tree.identify_row(event.y)
        
        if not item or not column:
            return
            
        # 不允许编辑序号列
        if column == '#1':  # 序号列
            return
            
        # 获取列和单元格的坐标
        x, y, w, h = self.tree.bbox(item, column)
        
        # 获取当前值
        col_num = int(column[1:]) - 1  # 将 #N 转换为索引 N-1
        values = self.tree.item(item)['values']
        current_value = str(values[col_num]) if values and col_num < len(values) else "0.0"
        
        # 配置编辑框
        self.edit_entry.delete(0, tk.END)
        self.edit_entry.insert(0, current_value)
        
        # 调整编辑框大小和位置
        padding = 4  # 增加padding值到4
        extra_width = 20  # 额外的宽度
        self.edit_entry.place(x=x-padding, 
                            y=y-padding,
                            width=w+padding*2+extra_width,  # 增加更多宽度
                            height=h+padding*2)  # 增加高度
        self.edit_entry.focus_set()
        self.edit_entry.select_range(0, tk.END)  # 选中所有文本
        
        # 保存当前编辑的单元格信息
        self.edit_entry.item = item
        self.edit_entry.column = col_num
        self.edit_entry.column_id = column
        self.edit_entry.values = list(values)

    def finish_edit(self, event):
        """完成编辑并更新数据"""
        if not self.edit_entry.winfo_ismapped():
            return
            
        try:
            # 获取新值并尝试转换为浮点数
            input_value = self.edit_entry.get().strip()
            new_value = float(input_value) if input_value else 0.0
            
            # 获取当前编辑的列名
            col_index = self.edit_entry.column
            col_name = self.table_columns[col_index]
            
            # 序号列不应该被编辑
            if col_name == '序号':
                self.edit_entry.place_forget()
                return
            
            # 获取对应的电机ID和限制范围
            motor_id = get_motor_id(col_name)
            if motor_id:
                joint_info = get_joint_info(motor_id)
                if joint_info:
                    min_angle, max_angle = joint_info['range']
                    # 限制角度范围
                    new_value = min(max(new_value, min_angle), max_angle)
            
            # 格式化为一位小数
            formatted_value = f"{new_value:.1f}"
            
            # 更新表格数据
            values = list(self.edit_entry.values)
            values[col_index] = formatted_value
            self.tree.item(self.edit_entry.item, values=values)
            
        except ValueError:
            messagebox.showerror("错误", "请输入有效的数字")
        finally:
            self.edit_entry.place_forget()
            
    def cancel_edit(self, event):
        """取消编辑"""
        self.edit_entry.place_forget()
        return "break"

    def refresh_ports(self):
        """刷新串口列表"""
        if hasattr(self, 'port_combobox'):
            available_ports = self.hand_api.get_available_ports()
            if available_ports:
                self.port_combobox['values'] = available_ports
                if not self.port_combobox.get():  # 如果当前没有选中的值
                    self.port_combobox.set(available_ports[0])

    def toggle_connection(self):
        """切换串口连接状态"""
        if not self.is_connected:
            self.connect_motor()
        else:
            self.disconnect_motor()
            
    def toggle_all_torque(self):
        """切换所有电机的使能状态"""
        if not self.is_connected:
            return
            
        success, new_state = self.hand_api.toggle_all_torque()
        if success:
            # 根据新状态更新按钮文本和控件状态
            if new_state:
                self.torque_btn.configure(text="关闭电机出力")
                self.enable_operation_controls()
            else:
                self.torque_btn.configure(text="开启电机出力")
                # 禁用除了出力按钮之外的所有控件
                self.open_hand_btn.state(['disabled'])
                self.velocity_slider.state(['disabled'])
                self.run_all_btn.state(['disabled'])
                self.stop_btn.state(['disabled'])
                for motor_id in self.motors:
                    if 'slider' in self.motors[motor_id]:
                        self.motors[motor_id]['slider'].state(['disabled'])
                    if 'entry' in self.motors[motor_id]:
                        self.motors[motor_id]['entry'].state(['disabled'])
            
            # 强制更新界面
            self.root.update()
            # 确保出力按钮始终可用
            self.torque_btn.state(['!disabled'])
            
            # 打印调试信息
            print(f"按钮状态已更新: {'启用' if new_state else '禁用'}")
            print(f"按钮文本: {self.torque_btn.cget('text')}")
        else:
            messagebox.showerror("错误", "切换电机出力状态失败")
            
    def connect_motor(self):
        """连接电机并更新界面状态"""
        selected_port = self.port_combobox.get()
        if not selected_port:
            messagebox.showerror("错误", "请选择串口")
            return
            
        success, message = self.hand_api.connect_motor(selected_port)
        if success:
            self.is_connected = True
            self.connect_btn.configure(text="断开")
            self.port_combobox.state(['disabled'])
            
            # 启用校准按钮
            self.calibrate_btn.state(['!disabled'])
            
            # 检查电机的实际出力状态
            try:
                # 检查第一个电机的状态作为参考
                torque_enabled, _, _ = self.hand_api.packetHandler.read1ByteTxRx(
                    self.hand_api.portHandler, 1, ADDR_TORQUE_ENABLE)
                # 根据实际状态设置按钮文本
                self.torque_btn.configure(text="关闭电机出力" if torque_enabled == TORQUE_ENABLE else "开启电机出力")
            except:
                # 如果读取失败，默认显示"开启电机出力"
                self.torque_btn.configure(text="开启电机出力")
            
            # 启用出力按钮
            self.torque_btn.state(['!disabled'])
            
            # 检查电机校准状态
            uncalibrated_motors = self.hand_api.get_uncalibrated_motors()
            if uncalibrated_motors:
                warning_message = f"警告：以下电机需要重新校准 :\n{uncalibrated_motors}\n\n建议立即运行手部校准程序。"
                messagebox.showwarning("校准提示", warning_message)
                self.disable_operation_controls()
                self.calibrate_btn.state(['!disabled'])  # 确保校准按钮可用
            else:
                self.enable_operation_controls()  # 启用所有操作控件
                # 连接成功后立即读取当前位置
                self.read_current_positions()
                
            # 强制更新界面
            self.root.update_idletasks()
        else:
            messagebox.showerror("错误", f"连接失败: {message}")
            self.is_connected = False

    def disconnect_motor(self):
        """断开电机连接并更新界面状态"""
        if self.is_connected:
            self.hand_api.disconnect_motor()
            
        # 重置连接状态和界面
        self.is_connected = False
        self.connect_btn.configure(text="连接")
        self.port_combobox.state(['!disabled'])
        self.disable_operation_controls()
        self.torque_btn.configure(text="开启电机出力")

    def calibrate_hand(self):
        """
        手部校准功能
        提示用户将五指张开，然后进行校准
        """
        if not self.is_connected:
            messagebox.showerror("错误", "请先连接串口")
            return

        # 密码验证
        password = simpledialog.askstring("密码验证", "请输入校准密码：", show='*')
        if not password:  # 用户取消输入
            return
            
        # 禁用所有操作按钮，避免校准过程中的干扰
        self.disable_operation_controls()
        
        # 先关闭电机出力
        self.hand_api.set_all_torque(False)
        self.torque_btn.configure(text="开启电机出力")
        
        # 提示用户将五指张开
        message = """校准步骤：

1. 已关闭电机出力，现在可以手动调整手指位置

2. 请将所有手指完全张开：
   - 所有手指完全伸直
   - 拇指完全展开
   - 所有关节处于中立位置

3. 确认所有手指都已张开后，点击"是"开始校准

注意：校准时请保持手指位置不变"""

        if not messagebox.askyesno("校准提示", message):
            # 用户取消校准，重新启用电机出力和按钮
            self.hand_api.set_all_torque(True)
            self.torque_btn.configure(text="关闭电机出力")
            self.enable_operation_controls()
            return
            
        # 执行校准
        success, message = self.hand_api.calibrate_hand(password)
        
        if success:
            # API调用成功，现在验证电机位置是否为180度
            print("校准API调用成功，正在验证电机位置...")
            time.sleep(0.5)  # 等待电机稳定
            
            current_motor_angles = self.hand_api.read_current_positions()
            
            failed_motors = []
            
            if not current_motor_angles:
                messagebox.showerror("校准验证失败", "无法读取任何电机位置进行验证。")
                self.disable_operation_controls()
                return

            # 遍历所有在线的电机，检查其位置
            for motor_id, angle in current_motor_angles.items():
                if not (179.5 <= angle <= 180.5):
                    failed_motors.append(motor_id)

            if failed_motors:
                error_msg = f"校准失败：以下电机的校准后位置不正确:\n{failed_motors}\n\n请重启电源后重新校准。"
                messagebox.showerror("校准验证失败", error_msg)
                self.disable_operation_controls()
                self.calibrate_btn.state(['!disabled']) # 确保校准按钮可用
            else:
                # 校准成功且验证通过
                success_msg = f"{message}\n所有关节电机已验证成功。"
                messagebox.showinfo("成功", success_msg)
                self.hand_api.set_all_torque(True)
                self.torque_btn.configure(text="关闭电机出力")
                self.enable_operation_controls()
        else:
            # API调用本身失败
            messagebox.showerror("错误", message)
            self.disable_operation_controls()
            self.calibrate_btn.state(['!disabled']) # 确保校准按钮可用

    def disable_operation_controls(self):
        """禁用除了连接和校准按钮之外的所有操作按钮"""
        # 手势控制按钮
        self.open_hand_btn.state(['disabled'])
        
        # 速度控制
        self.velocity_slider.state(['disabled'])
        
        # 所有电机控制滑块和输入框
        for motor_id in self.motors:
            if 'slider' in self.motors[motor_id]:
                try:
                    self.motors[motor_id]['slider'].state(['disabled'])
                    # 重置滑块位置到0
                    self.motors[motor_id]['slider'].set(0)
                except Exception as e:
                    print(f"禁用电机 {motor_id} 滑块时出错: {str(e)}")
            if 'entry' in self.motors[motor_id]:
                try:
                    self.motors[motor_id]['entry'].state(['disabled'])
                    # 重置输入框值到0
                    self.motors[motor_id]['entry'].delete(0, tk.END)
                    self.motors[motor_id]['entry'].insert(0, "0.0")
                except Exception as e:
                    print(f"禁用电机 {motor_id} 输入框时出错: {str(e)}")
        
        # 动作序列控制按钮
        self.run_all_btn.state(['disabled'])
        self.stop_btn.state(['disabled'])
        
        # 出力按钮保持启用状态
        self.torque_btn.state(['!disabled'])

    def enable_operation_controls(self):
        """启用所有操作按钮"""
        if self.is_connected:
            # 手势控制按钮
            self.open_hand_btn.state(['!disabled'])
            
            # 速度控制
            self.velocity_slider.state(['!disabled'])
            
            # 所有电机控制滑块和输入框
            for motor_id in self.motors:
                if 'slider' in self.motors[motor_id]:
                    try:
                        self.motors[motor_id]['slider'].state(['!disabled'])
                    except Exception as e:
                        print(f"启用电机 {motor_id} 滑块时出错: {str(e)}")
                if 'entry' in self.motors[motor_id]:
                    try:
                        self.motors[motor_id]['entry'].state(['!disabled'])
                    except Exception as e:
                        print(f"启用电机 {motor_id} 输入框时出错: {str(e)}")
            
            # 动作序列控制按钮
            self.run_all_btn.state(['!disabled'])
            self.stop_btn.state(['!disabled'])
            
            # 出力按钮
            self.torque_btn.state(['!disabled'])
        else:
            # 如果没有校准文件或未连接，保持控件禁用状态
            self.disable_operation_controls()
            
        # 校准按钮始终可用（如果已连接）
        if self.is_connected:
            self.calibrate_btn.state(['!disabled'])

    def read_current_positions(self):
        """读取所有启用电机的当前位置"""
        if not self.is_connected:
            messagebox.showerror("错误", "请先连接串口")
            return
            
        # 读取当前位置
        positions = self.hand_api.read_current_positions()
        if not positions:
            messagebox.showerror("错误", "检查电源是否正常连接，或者重新上电")
            return
            
        # 更新界面显示
        for motor_id, motor_angle in positions.items():
            if motor_id in self.motors:
                # 转换为关节角度
                joint_angle = self.hand_api.motor_to_joint_angle(motor_id, motor_angle)
                # 更新滑块和输入框的值，保留一位小数
                self.motors[motor_id]['slider'].set(joint_angle)
                self.motors[motor_id]['entry'].delete(0, tk.END)
                self.motors[motor_id]['entry'].insert(0, f"{joint_angle:.1f}")
                # 更新当前位置显示
                self.motors[motor_id]['current_pos'].configure(text=f"当前位置: {joint_angle:.1f}°")
                print(f"电机 {motor_id} - 电机角度: {motor_angle:.1f}°, 关节角度: {joint_angle:.1f}°")

    def save_current_positions(self):
        """保存当前所有关节角度"""
        if not self.is_connected:
            messagebox.showerror("错误", "请先连接串口")
            return
            
        # 获取当前位置
        positions = self.hand_api.save_current_positions()
        if not positions:
            messagebox.showerror("错误", "读取电机位置失败")
            return
            
        # 生成新的序号
        items = self.tree.get_children()
        new_index = len(items) + 1
        
        # 准备表格数据
        values = [str(new_index)]  # 序号列
        for col in self.table_columns[1:]:  # 跳过序号列
            motor_id = get_motor_id(col)
            if motor_id in positions:
                values.append(f"{positions[motor_id]:.1f}")
            else:
                values.append("0.0")
                
        # 插入新行
        self.tree.insert('', 'end', values=values)
        
        # 更新界面显示
        for motor_id, joint_angle in positions.items():
            if motor_id in self.motors:
                # 更新滑块和输入框的值，保留一位小数
                self.motors[motor_id]['slider'].set(joint_angle)
                self.motors[motor_id]['entry'].delete(0, tk.END)
                self.motors[motor_id]['entry'].insert(0, f"{joint_angle:.1f}")
                # 更新当前位置显示
                self.motors[motor_id]['current_pos'].configure(text=f"当前位置: {joint_angle:.1f}°")
                print(f"电机 {motor_id} - 关节角度: {joint_angle:.1f}°")

    def on_slider_change(self, motor_id, value):
        """处理滑块值变化事件"""
        if not self.is_connected:
            return
            
        try:
            joint_angle = float(value)
            # 先限制关节角度范围
            limited_joint_angle = self.hand_api.limit_angle(motor_id, joint_angle)
            # 将关节角度转换为电机角度
            motor_angle = self.hand_api.joint_to_motor_angle(motor_id, limited_joint_angle)
            
            # 更新输入框显示（显示关节角度）
            self.motors[motor_id]['entry'].delete(0, tk.END)
            self.motors[motor_id]['entry'].insert(0, f"{limited_joint_angle:.1f}")
            
            # 设置电机位置
            if not self.hand_api.set_motor_position(motor_id, motor_angle):
                messagebox.showerror("错误", f"设置电机 {motor_id} 位置失败")
                
        except ValueError:
            pass

    def on_entry_change(self, motor_id):
        """处理输入框值变化事件"""
        if not self.is_connected:
            return
            
        try:
            value = self.motors[motor_id]['entry'].get().strip()
            joint_angle = float(value) if value else 0.0
            
            # 先限制关节角度范围
            limited_joint_angle = self.hand_api.limit_angle(motor_id, joint_angle)
            # 将关节角度转换为电机角度
            motor_angle = self.hand_api.joint_to_motor_angle(motor_id, limited_joint_angle)
            
            # 更新滑块
            self.motors[motor_id]['slider'].set(limited_joint_angle)
            
            # 设置电机位置
            if not self.hand_api.set_motor_position(motor_id, motor_angle):
                messagebox.showerror("错误", f"设置电机 {motor_id} 位置失败")
                
        except ValueError:
            messagebox.showerror("错误", "请输入有效的数字")
            # 重置为当前位置
            self.read_current_positions()

    def run_all_sequences(self, interval, loop_count, selected_items):
        """执行表格中的所有动作"""
        if not self.is_connected:
            messagebox.showerror("错误", "请先连接串口")
            return
            
        try:
            # 获取所有行
            items = self.tree.get_children()
            if not items:
                messagebox.showwarning("警告", "表格中没有动作")
                return
                
            # 禁用控制按钮
            self.disable_operation_controls()
            self.run_all_btn.state(['disabled'])
            self.stop_btn.state(['!disabled'])
            
            # 创建并启动动作执行线程
            self.action_thread = threading.Thread(target=self._run_sequences, args=(interval, loop_count, selected_items))
            self.action_thread.daemon = True
            self.action_thread.start()
            
        except Exception as e:
            print(f"执行动作序列时出错: {str(e)}")
            self.enable_operation_controls()

    def update_motor_ui(self, motor_id, joint_angle):
        """更新电机UI显示"""
        if motor_id in self.motors:
            # 更新滑块和输入框的值，保留一位小数
            self.motors[motor_id]['slider'].set(joint_angle)
            self.motors[motor_id]['entry'].delete(0, tk.END)
            self.motors[motor_id]['entry'].insert(0, f"{joint_angle:.1f}")
            # 更新当前位置显示
            self.motors[motor_id]['current_pos'].configure(text=f"当前位置: {joint_angle:.1f}°")

    def _run_sequences(self, interval, loop_count, selected_items):
        """在单独的线程中执行动作序列"""
        try:
            # 启用停止按钮
            self.root.after(0, lambda: self.stop_btn.state(['!disabled']))
            
            # 更新循环计数标签的总循环次数
            self.root.after(0, lambda: self.loop_count_label.configure(text=f"循环次数: {loop_count}/当前循环: 0"))
            
            for current_loop in range(loop_count):
                if self.stop_flag:
                    break
                
                # 更新当前循环次数显示
                self.root.after(0, lambda c=current_loop+1: self.loop_count_label.configure(text=f"循环次数: {loop_count}/当前循环: {c}"))
                
                for item in selected_items:
                    if self.stop_flag:
                        break
                        
                    values = self.tree.item(item)['values']
                    positions = {}
                    
                    # 收集所有电机的目标位置
                    for col_index, col_name in enumerate(self.table_columns[1:], 1):
                        motor_id = get_motor_id(col_name)
                        if motor_id:
                            try:
                                joint_angle = float(values[col_index])
                                # 先限制关节角度范围
                                limited_joint_angle = self.hand_api.limit_angle(motor_id, joint_angle)
                                # 将关节角度转换为电机角度
                                motor_angle = self.hand_api.joint_to_motor_angle(motor_id, limited_joint_angle)
                                positions[motor_id] = motor_angle
                                # 更新UI显示（使用原始关节角度）
                                self.root.after(0, lambda m=motor_id, a=joint_angle: self.update_motor_ui(m, a))
                            except (ValueError, IndexError):
                                continue
                    
                    # 设置所有电机位置
                    if positions:
                        if not self.hand_api.set_motors_position_sync(positions):
                            messagebox.showerror("错误", "设置电机位置失败")
                            break
                        time.sleep(interval)  # 等待动作完成
                    
        except Exception as e:
            print(f"执行动作序列时出错: {str(e)}")
        finally:
            self.stop_flag = False
            # 重置循环计数显示
            self.root.after(0, lambda: self.loop_count_label.configure(text="循环次数: 0/当前循环: 0"))
            # 禁用停止按钮
            self.root.after(0, lambda: self.stop_btn.state(['disabled']))
            self.root.after(0, self.enable_operation_controls)

    def stop_sequences(self):
        """停止执行动作序列"""
        print("停止序列执行")  # 调试信息
        self.stop_flag = True
        # 禁用停止按钮
        self.stop_btn.state(['disabled'])

    def open_hand(self):
        """五指张开功能"""
        if not self.is_connected:
            messagebox.showerror("错误", "请先连接串口")
            return
            
        if not self.hand_api.open_hand():
            messagebox.showerror("错误", "执行张开手势失败")

    def on_velocity_change(self, value):
        """处理速度滑块值变化事件"""
        try:
            velocity = int(float(value))
            if hasattr(self, 'velocity_label') and self.velocity_label is not None:
                self.velocity_label.configure(text=str(velocity))
            if self.is_connected:
                if not self.hand_api.set_all_velocity_sync(velocity):
                    messagebox.showerror("错误", "设置电机速度失败")
        except ValueError:
            pass

    def run_selected_row(self):
        """执行选中行的动作"""
        if not self.is_connected:
            messagebox.showerror("错误", "请先连接串口")
            return
            
        selected_items = self.tree.selection()
        if not selected_items:
            messagebox.showwarning("警告", "没有选中任何行")
            return
            
        self.run_all_sequences(DEFAULT_INTERVAL, 1, selected_items)

    def save_sequence_data(self):
        """保存动作序列数据到文件"""
        if not self.tree.get_children():
            messagebox.showwarning("警告", "表格中没有数据可保存")
            return
            
        # 让用户选择保存位置
        file_path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            title="保存动作序列数据"
        )
        
        if not file_path:  # 用户取消了保存
            return
            
        try:
            with open(file_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                # 写入表头
                writer.writerow(self.table_columns)
                # 写入数据
                for item in self.tree.get_children():
                    writer.writerow(self.tree.item(item)['values'])
            messagebox.showinfo("成功", "动作序列数据已保存")
        except Exception as e:
            messagebox.showerror("错误", f"保存文件时出错：{str(e)}")

    def load_sequence_data(self):
        """从文件加载动作序列数据"""
        # 让用户选择要加载的文件
        file_path = filedialog.askopenfilename(
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            title="加载动作序列数据"
        )
        
        if not file_path:  # 用户取消了选择
            return
            
        try:
            with open(file_path, 'r', newline='', encoding='utf-8') as f:
                reader = csv.reader(f)
                # 跳过表头
                next(reader)
                # 清空当前表格
                self.clear_table()
                # 读入数据
                for row in reader:
                    self.tree.insert('', 'end', values=row)
            messagebox.showinfo("成功", "动作序列数据已加载")
        except UnicodeDecodeError:
            # 如果 UTF-8 解码失败，尝试使用 GBK 编码
            try:
                with open(file_path, 'r', newline='', encoding='gbk') as f:
                    reader = csv.reader(f)
                    # 跳过表头
                    next(reader)
                    # 清空当前表格
                    self.clear_table()
                    # 读入数据
                    for row in reader:
                        self.tree.insert('', 'end', values=row)
                messagebox.showinfo("成功", "动作序列数据已加载")
            except Exception as e:
                messagebox.showerror("错误", f"加载文件时出错：{str(e)}")
        except Exception as e:
            messagebox.showerror("错误", f"加载文件时出错：{str(e)}")

    def delete_selected_row(self):
        """删除选中的表格行"""
        selections = self.tree.selection()
        if not selections:
            messagebox.showwarning("警告", "请先选择要删除的行")
            return
            
        if messagebox.askyesno("确认", "确定要删除选中的行吗？"):
            # 删除选中的行
            for item in selections:
                self.tree.delete(item)
            # 重新编号
            self.renumber_rows()

    def clear_table(self):
        """清空表格"""
        if not self.tree.get_children():
            messagebox.showinfo("提示", "表格已经是空的")
            return
            
        if messagebox.askyesno("确认", "确定要清空表格吗？"):
            for item in self.tree.get_children():
                self.tree.delete(item)

    def renumber_rows(self):
        """重新为表格行编号"""
        items = self.tree.get_children()
        for i, item in enumerate(items, 1):
            values = list(self.tree.item(item)['values'])
            values[0] = str(i)  # 更新序号
            self.tree.item(item, values=values)

def main():
    """主函数"""
    # Set display environment
    import os
    import sys
    
    # Try different display configurations
    display_configs = [':0', ':1', 'localhost:0', 'localhost:1']
    display_set = False
    
    for display in display_configs:
        try:
            os.environ['DISPLAY'] = display
            # Test if display is available
            test_root = tk.Tk()
            test_root.destroy()
            display_set = True
            break
        except:
            continue
    
    if not display_set:
        print("警告: 无法连接到显示服务器，尝试使用虚拟显示...")
        try:
            # Try to use virtual display
            os.environ['DISPLAY'] = ':0'
            os.environ['XAUTHORITY'] = '/home/' + os.getenv('USER') + '/.Xauthority'
        except:
            print("错误: 无法设置显示环境，程序可能无法正常显示GUI界面")
    
    # Create the main window
    try:
        root = tk.Tk()
        # 使用 ttkbootstrap 创建主窗口
        style = Style(theme='darkly')
        style.master = root
    except Exception as e:
        print(f"错误: 无法创建主窗口: {str(e)}")
        sys.exit(1)
    
    # 设置程序图标
    try:
        # 获取当前脚本所在目录的绝对路径
        script_dir = os.path.dirname(os.path.abspath(__file__))
        
        # 根据操作系统选择图标格式
        if sys.platform.startswith('linux'):
            # Linux系统优先使用PNG格式
            png_path = os.path.join(script_dir, 'app_icon.png')
            if os.path.exists(png_path):
                icon = tk.PhotoImage(file=png_path)
                root.iconphoto(True, icon)
            else:
                print("警告：未找到PNG格式图标文件")
        elif sys.platform.startswith('win'):
            # Windows系统优先使用ICO格式
            ico_path = os.path.join(script_dir, 'app_icon.ico')
            if os.path.exists(ico_path):
                root.iconbitmap(ico_path)
            else:
                # 如果找不到ICO文件，尝试使用PNG
                png_path = os.path.join(script_dir, 'app_icon.png')
                if os.path.exists(png_path):
                    icon = tk.PhotoImage(file=png_path)
                    root.iconphoto(True, icon)
                else:
                    print("警告：未找到图标文件")
        else:
            # 其他系统尝试两种格式
            png_path = os.path.join(script_dir, 'app_icon.png')
            ico_path = os.path.join(script_dir, 'app_icon.ico')
            if os.path.exists(png_path):
                icon = tk.PhotoImage(file=png_path)
                root.iconphoto(True, icon)
            elif os.path.exists(ico_path):
                root.iconbitmap(ico_path)
            else:
                print("警告：未找到图标文件")
    except Exception as e:
        print(f"设置程序图标失败: {str(e)}")
    
    app = MultiMotorControlGUI(root)
    
    # 关闭窗口时的清理操作
    def on_closing():
        if app.is_connected:
            app.hand_api.disconnect_motor()
        root.destroy()
    
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()

if __name__ == "__main__":
    main() 