# 1. **概述**

灵心巧手，创造万物。

LinkerHand 灵巧手 ROS SDK 是由灵心巧手（北京）科技有限公司开发的一款软件工具，用于驱动其灵巧手系列产品，并提供功能示例。它支持多种设备（如笔记本、台式机、树莓派、Jetson 等），主要服务于人型机器人、工业自动化和科研院所等领域，适用于人型机器人、柔性化生产线、具身大模型训练和数据采集等场景。

# 2. **警告**

1. 请保持远离灵巧手活动范围，避免造成人身伤害或设备损坏。

2. 执行动作前请务必进行安全评估，以防止发生碰撞。

3. 请保护好灵巧手。

# 3. **版本说明**
V1.0.1
1. 支持L30线驱版本LinkeerHand灵巧手
2. GUI控制界面
3. Python Demo示例


3. 新增GUI控制界面

# 4. 准备工作

## 4.1 系统与硬件需求

* 操作系统：Ubuntu24.04

* ROS2版本：Jazzy

* Python版本：V3.13.2

* 硬件：amd64_x86/arm64 配备 5v标准USB接口

## 4.2 下载

```bash
$ mkdir -p linker_hand_l30_sdk/src    #创建目录
$ cd linker_hand_l30_sdk/src    #进入目录
$ git clone https://github.com/linkerbotai/linker_hand_l30_sdk.git    #获取SDK
```

## 4.3 确认硬件USB端口，修改配置文件
- 接通电源，USB插在上位机上后，查询USB端口。确定端口后修改linker_hand_l30.launch.py(linker_hand_l30/launch/linker_hand_l30.launch.py)13行，修改为查询到的端口
```bash
$ ls /dev/ttyUSB*
$/dev/ttyUSB0  # 说明设备识别在USB0端口上
$ sudo chmod 777 /dev/ttyUSB0  # 给端口权限
$ sudo vim linker_hand_l30/launch/linker_hand_l30.launch.py
```

## 4.4 安装依赖与编译

```bash
$ cd linker_hand_l30_sdk/src/linker_hand_l30_sdk    #进入目录
$ pip install -r requirements.txt    #安装所需依赖
$ pip install l30_hand_api-1.0.0-py3-none-any.whl
$ cd linker_hand_l30_sdk # 回到工程目录
$ colcon build --symlink-install    #编译和构建ROS包
```

# 5 启动SDK
```bash
$ cd linker_hand_l30_sdk
$ source ./install/setup.bash
$ ros2 launch linker_hand_l30 linker_hand_l30.launch.py
$ [linker_hand_l30_node-1] 2025-06-26 15:45:43  连接手部设备成功
```
## 5.1启动GUI控制L30灵巧手
 - 启动SDK后，新开一个终端。注:由于有界面，不能使用ssh远程连接开启GUI
```bash
$ cd linker_hand_l30_sdk
$ source ./install/setup.bash
$ ros2 launch gui_control gui_control.launch.py
```

# 6 TOPIC说明
```bash
/cb_right_hand_control_cmd # 控制右手运动话题  (0~90)度S
```
## 6.1 position说明
```bash
['大拇指侧摆', '拇指旋转', '拇指弯曲', '拇指指尖', '食指侧摆', '食指指根弯曲', '食指指尖', '中指侧摆', '中指指根', '中指指尖', '无名指侧摆', '无名指指根', '无名指指尖', '小指侧摆', '小指指根', '小指指尖', '手腕']
```
