# 1. **概述**

灵心巧手，创造万物。

LinkerHand 灵巧手 ROS SDK 是由灵心巧手（北京）科技有限公司开发的一款软件工具，用于驱动其灵巧手系列产品，并提供功能示例。它支持多种设备（如笔记本、台式机、树莓派、Jetson 等），主要服务于人型机器人、工业自动化和科研院所等领域，适用于人型机器人、柔性化生产线、具身大模型训练和数据采集等场景。

# 2. **警告**

1. 请保持远离灵巧手活动范围，避免造成人身伤害或设备损坏。

2. 执行动作前请务必进行安全评估，以防止发生碰撞。

3. 请保护好灵巧手。

# 3. **版本说明**
V2.0.1
1. 更新GUI控制和动作序列


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

* 硬件：amd64_x86/arm64 配备 5v标准USB接口


## L30 CANFD请跳转至[L30 CANFD](https://github.com/linker-bot/linkerhand-l30-sdk/tree/v2.0.0)

## 4.2 下载 

```bash
$ mkdir -p linker_hand_l30_sdk/src    #创建目录
$ cd linker_hand_l30_sdk/src    #进入目录
$ git clone https://github.com/linkerbotai/linker_hand_l30_sdk.git    #获取SDK
```

## 4.3 确认硬件USB端口，修改配置文件
- 接通电源，USB插在上位机上后，查询USB端口。确定端口后修改linker_hand_l30.launch.py(linker_hand_l30/launch/linker_hand_l30.launch.py)13行，修改为查询到的端口
- 修改linker_hand_l30/launch/linker_hand_l30.launch.py，按照参数说明修改左手 or 右手
```bash
$ ls /dev/ttyUSB*
$/dev/ttyUSB0  # 说明设备识别在USB0端口上
$ sudo chmod 777 /dev/ttyUSB0
$ sudo vim linker_hand_l30/launch/linker_hand_l30.launch.py
```

## 4.4 安装依赖与编译

```bash
$ cd linker_hand_l30_sdk/src/linker_hand_l30_sdk    #进入目录
$ pip install -r requirements.txt    #安装所需依赖
$ pip install l30_hand_api-1.0.4-py3-none-any.whl
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
 - 修改gui_control/launch/gui_control.launch.py 按照参数说明，修改左手 or 右手
```bash
$ cd linker_hand_l30_sdk
$ source ./install/setup.bash
$ ros2 launch gui_control gui_control.launch.py
```
<img  src="resource/gui.png" width="550">

- 增加或修改动作示例。在[constants.py](https://github.com/linker-bot/linkerhand-l30-sdk/blob/main/gui_control/gui_control/config/constants.py)文件中可增加或修改动作。
```python
# 例如增加L30的动作序列
"L30": HandConfig(
        joint_names=[
            "大拇指侧摆", "拇指旋转", "拇指弯曲", "拇指指尖",
            "食指侧摆", "食指指根弯曲", "食指指尖",
            "中指侧摆", "中指指根", "中指指尖",
            "无名指侧摆", "无名指指根", "无名指指尖",
            "小指侧摆", "小指指根", "小指指尖",
            "手腕"
        ],
        init_pos=[0] * 17,
        # ..........
        preset_actions={
            "张开": [16, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            "握拳": [10, 48, 55, 53, -7, 61, 80, 0, 74, 65, 0, 72, 80, 0, 71, 80, 0],
            "壹": [10, 48, 55, 53, -7, 0, 0, 0, 74, 65, 0, 72, 80, 0, 71, 80, 0],
            "贰": [0, 48, 54, 53, 5, 0, 0, -6, 0, 0, 0, 72, 80, 0, 71, 80, 0],
            # 这里增加新的动作序列.......
        }
    ),
```


# 6 TOPIC说明
```bash
/cb_right_hand_control_cmd # 控制右手运动话题  (0~90)度
/cb_hand_setting_cmd #设置手指运行速度
```
## 6.1设置速度示例
```python
self.setting_pub = self.create_publisher(String, f'/cb_hand_setting_cmd', 10)
def _hand_setting_cb(self):
        dic = {
            "setting_cmd": "set_speed",
            "params": {
                "speed": 255  # 设置速度为100
            }
        }
        msg = String()
        msg.data = json.dumps(dic)
        self.setting_pub.publish(msg)
```
## 6.2 position说明
```bash
['大拇指侧摆', '拇指旋转', '拇指弯曲', '拇指指尖', '食指侧摆', '食指指根弯曲', '食指指尖', '中指侧摆', '中指指根', '中指指尖', '无名指侧摆', '无名指指根', '无名指指尖', '小指侧摆', '小指指根', '小指指尖', '手腕']
```
