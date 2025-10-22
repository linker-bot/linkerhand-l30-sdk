# 1. **概述**

灵心巧手，创造万物。

LinkerHand 灵巧手 ROS SDK 是由灵心巧手（北京）科技有限公司开发的一款软件工具，用于驱动其灵巧手系列产品，并提供功能示例。它支持多种设备（如笔记本、台式机、树莓派、Jetson 等），主要服务于人型机器人、工业自动化和科研院所等领域，适用于人型机器人、柔性化生产线、具身大模型训练和数据采集等场景。

# 2. **警告**

1. 请保持远离灵巧手活动范围，避免造成人身伤害或设备损坏。

2. 执行动作前请务必进行安全评估，以防止发生碰撞。

3. 请保护好灵巧手。

# 3. **版本说明**

V2.0.1
1. 支持L30线驱版本支持CANFD

V1.0.2
1. 支持L30线驱版本LinkeerHand灵巧手左手和右手
2. 支持手指运行速度设置

V1.0.1
1. 支持L30线驱版本LinkeerHand灵巧手
2. GUI控制界面
3. Python Demo示例


3. 新增GUI控制界面

# 4. 准备工作

## 4.1 系统与硬件需求

* 操作系统：Ubuntu24.04

* ROS2版本：Jazzy

* Python版本：V3.12

* 硬件：amd64_x86/arm64 配备 USB CANFD

## 4.1.1 L30绳驱CANFD版本(红色电机)准备工作 USB U2D2版本(黑色电机)请跳转至4.2

将libcanbus用命令解压到/usr/local/lib/目录下面

tar -xvf libcanbus.tar -C /usr/local/lib/

配置环境变量:      export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib

libcanbus.a为静态库，libcanbus.so为共享库

libcanbus_arm        编译器为    arm-linux-gnueabihf-gcc
libcanbus_arm64      编译器为    aarch64-linux-gnu-gcc
libcanbus(ubuntu20)  编译器为    gcc version 9.4.0
libcanbus(ubuntu22)  编译器为    gcc version 11.3.0


编译时先加载libcanbus库，再加载libusb库(源码注释里面有编译方法)。

如果编译提示pthread相关错误，说明libcanbus.tar
自带的libusb库不匹配， 请安装libusb库        sudo apt-get install libusb-1.0-0-dev 并删掉/usr/local/lib目录下的libusb相关库文件

如果提示ludev错误，请安装libudev-dev：       sudo apt-get install libudev-dev

如果编译过程中找不到cc1plus 请安装           sudo apt-get install --reinstall build-essential


注意：root用户可直接读写usbcan设备，非root用户，需要修改usbcan模块的操作权限，可百度搜索修改方法。
或者尝试将99-canfd.rules文件放到 /etc/udev/rules.d/， 然后执行
```bash
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger
```
重启系统。

(linux下面不能识别为独立双通道，一个双通道设备，有两个通道0和1，0对应can1，1对应can2)


## 4.2 下载

```bash
$ mkdir -p linker_hand_l30_sdk/src    #创建目录
$ cd linker_hand_l30_sdk/src    #进入目录
$ # 如果是USB D to D版本请clone V1.0.5版本
$ git clone -b v1.0.5 https://github.com/linker-bot/linkerhand_l30_sdk.git
$ # 获取CANFD版本
$ git clone https://github.com/linker-bot/linkerhand_l30_sdk.git
$ sudo chmod a+x src/linkerhand_l30_sdk/linker_hand_l30/linker_hand_l30/linker_hand_l30.py # 添加执行权限
```

## 4.3 确认硬件USB端口，修改配置文件
- 接通电源，USB转CANFD插在上位机上
- 修改linker_hand_l30/launch/linker_hand_l30.launch.py，按照参数说明修改左手 or 右手

## 4.4 安装依赖与编译

```bash
$ cd linker_hand_l30_sdk/src/linker_hand_l30_sdk    #进入目录
$ pip install -r requirements.txt    #安装所需依赖
$ cd linker_hand_l30_sdk # 回到工程目录
$ colcon build --symlink-install    #编译和构建ROS包
```

# 5 启动SDK
```bash
$ cd linker_hand_l30_sdk
$ source ./install/setup.bash
$ ros2 launch linker_hand_l30 linker_hand_l30.launch.py
$ # 显示一下信息则连接成功
$ 🔍 正在查询设备类型...
$ [linker_hand_l30-1]    查询右手设备 (ID: 0x01)...
$ [linker_hand_l30-1]      ✅ 设备 0x01 响应正常 (数据长度: 64)
$ [linker_hand_l30-1]      设备信息: L30-Hand-F, 类型: 右手
$ [linker_hand_l30-1] ✅ 检测到右手设备
$ [linker_hand_l30-1] ✅ 控制器连接成功，检测到设备类型: 右手
$ ........
```
## 5.1启动GUI控制L30灵巧手
 - 启动SDK后，新开一个终端。注:由于有界面，不能使用ssh远程连接开启GUI
 - 修改gui_control/launch/gui_control.launch.py 按照参数说明，修改左手 or 右手
```bash
$ cd linker_hand_l30_sdk
$ source ./install/setup.bash
$ ros2 launch gui_control gui_control.launch.py
```

# 6 TOPIC说明
```bash
/cb_right_hand_control_cmd # 控制右手运动话题  (0~255)
/cb_right_hand_info # 灵巧手硬件配置信息与软件设置信息
/cb_right_hand_state # 实时当前状态 0-255
/cb_right_hand_state_arc # 实时当前状态角度值
```
## 6.1 position说明
```bash
['大拇指侧摆', '拇指旋转', '拇指弯曲', '拇指指尖', '食指侧摆', '食指指根弯曲', '食指指尖', '中指侧摆', '中指指根', '中指指尖', '无名指侧摆', '无名指指根', '无名指指尖', '小指侧摆', '小指指根', '小指指尖', '手腕']
```


## 7 WIN使用方法
 - 接通电源，USB转CANFD插在WIN11上位机下直接运行L30FT灵巧手控制系统 for win11.exe即可
