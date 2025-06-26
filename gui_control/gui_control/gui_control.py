import sys
import rclpy,time,threading
from rclpy.node import Node
from std_msgs.msg import String, Header, Float32MultiArray
from sensor_msgs.msg import JointState

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QSlider,
    QLabel, QPushButton, QLineEdit, QGridLayout, QScrollArea
)

class HandControlNode(Node):
    def __init__(self):
        super().__init__('hand_control_node')
        # 声明参数（带默认值）
        self.declare_parameter('hand_type', 'right')
        self.declare_parameter('hand_joint', 'L30')
        self.declare_parameter('topic_hz', 30)
        self.declare_parameter('is_touch', False)
        
        # 获取参数值
        self.hand_type = self.get_parameter('hand_type').value
        self.hand_joint = self.get_parameter('hand_joint').value
        self.hz = self.get_parameter('topic_hz').value
        self.is_touch = self.get_parameter('is_touch').value
        self.last_msg = []
        self.publisher = self.create_publisher(JointState, f'/cb_{self.hand_type}_hand_control_cmd', 10)
    def get_hand(self):
        return self.hand_type,self.hand_joint,self.hz,self.is_touch
    def publish_control_cmd(self, msg):
        self.publisher.publish(msg)

class GuiApp(QWidget):
    handle_button_click = pyqtSignal(str)
    add_button_handle = pyqtSignal(str)

    def __init__(self,hand_type="left",hand_joint="L10",hz=30):
        super().__init__()
        self.hand_type = hand_type
        self.hand_joint = hand_joint
        self.interval = 1.0 / hz  # 每次间隔 0.033 秒
        self.last_msg = JointState()
        #self.yaml = LoadWriteYaml()
        self.setWindowTitle(f'ROS2 Control Linker Hand {hand_type} {hand_joint}')
        self.setFixedSize(800, 900)
        self.node = None
        self.buttons = []
        self.control_sliders = []
        self.slider_labels = {}
        self.row = 0
        self.column = 0
        self.BUTTONS_PER_ROW = 3  # 每行最多 3 个按钮

        main_layout = QHBoxLayout()
        self.setLayout(main_layout)

        # 左侧滑动条
        self.left_layout = QVBoxLayout()
        self.slider_list = ['Slider 1', 'Slider 2', 'Slider 3']
        self.init_hand()
        self.create_sliders(self.slider_list)
        left_widget = QWidget()
        left_widget.setLayout(self.left_layout)
        main_layout.addWidget(left_widget, alignment=Qt.AlignTop)

        
        


    def init_hand(self):
        if self.hand_joint == "L30":
            self.init_pos = [255] * 17
            self.joint_name = ['大拇指侧摆', '拇指旋转', '拇指弯曲', '拇指指尖', '食指侧摆', '食指指根弯曲', '食指指尖', '中指侧摆', '中指指根', '中指指尖', '无名指侧摆', '无名指指根', '无名指指尖', '小指侧摆', '小指指根', '小指指尖', '手腕']
        self.slider_list = self.joint_name

    def on_button_clicked(self,text):
        print("_-" * 20, flush=True)
        print(text, flush=True)

    def create_sliders(self, slider_names):
        joint_ranges = {
            "大拇指侧摆": (0, 40),
            "拇指旋转": (0, 80),
            "拇指弯曲": (0, 70),
            "拇指指尖": (0, 60),
            "食指侧摆": (-10, 10),
            "食指指根弯曲": (0, 80),
            "食指指尖": (0, 80),
            "中指侧摆": (-10, 10),
            "中指指根": (0, 80),
            "中指指尖": (0, 80),
            "无名指侧摆": (-10, 10),
            "无名指指根": (0, 80),
            "无名指指尖": (0, 80),
            "小指侧摆": (-10, 10),
            "小指指根": (0, 80),
            "小指指尖": (0, 80),
            "手腕": (-50, 50)
        }
        precision = 1
        for name in slider_names:
            slider = QSlider(Qt.Horizontal)
            if name not in joint_ranges:
                return
            # 缩放因子
            factor = 10 ** precision
            min_val, max_val = joint_ranges[name]
            int_min = int(min_val * factor)
            int_max = int(max_val * factor)

            # 创建滑动条
            slider = QSlider(Qt.Horizontal)
            slider.setRange(int_min, int_max)
            slider.setValue(int(0 * factor))
            slider.setTickPosition(QSlider.TicksBelow)
            slider.setTickInterval(factor)  # 例如 factor=10 表示步长0.1

            # 标签
            label = QLabel(f"{name} 0.0:")
            self.slider_labels[slider] = (label, name)

            # 连接信号
            slider.valueChanged.connect(self.slider_value_changed)

            # 垂直布局
            h_layout = QHBoxLayout()
            h_layout.addWidget(label)
            h_layout.addWidget(slider)

            item_widget = QWidget()
            item_widget.setLayout(h_layout)

            self.left_layout.addWidget(item_widget)
            self.control_sliders.append(slider)
            

    def slider_value_changed(self):
        all_values = [slider.value() for slider in self.control_sliders]
        all_values = [round(val / 10, 1) for val in all_values]

        sender_slider = self.sender()
        if sender_slider in self.slider_labels:
            label, name = self.slider_labels[sender_slider]
            label.setText(f"{name} {(sender_slider.value()/10)}:")

        self.last_msg = self.joint_state_msg(all_values)
        
    def loop_pub(self):
        self.thread_get_state = threading.Thread(target=self.pub_msg)
        self.thread_get_state.daemon = True
        self.thread_get_state.start()
    def pub_msg(self):
        while True:
            start = time.time()
            self.node.publish_control_cmd(self.last_msg)
            elapsed = time.time() - start
            time.sleep(max(0, self.interval - elapsed))


    def joint_state_msg(self, pose,vel=[]):
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.node.get_clock().now().to_msg()
        joint_state.name = []
        joint_state.position = [float(x) for x in pose]
        if len(vel) > 1:
            joint_state.velocity = [float(x) for x in vel]
        else:
            joint_state.velocity = [0.0] * len(pose)
        joint_state.effort = [0.0] * len(pose)
        return joint_state

    def add_button_to_right_layout(self):
        text = self.input_field.text().strip()
        if text:
            button = QPushButton(text)
            button.setFixedSize(100, 30)
            button.clicked.connect(lambda checked, text=text: self.handle_button_click.emit(text))

            # 添加到网格布局
            self.scroll_layout.addWidget(button, self.row, self.column, alignment=Qt.AlignLeft | Qt.AlignTop)

            # 更新行列位置
            self.column += 1
            if self.column >= self.BUTTONS_PER_ROW:
                self.column = 0
                self.row += 1

            self.input_field.clear()
            self.buttons.append(button)
            self.add_button_handle.emit(text)

def main(args=None):
    rclpy.init(args=args)
    node = HandControlNode()
    time.sleep(1)
    hand_type,hand_joint,hz,is_touch = node.get_hand()
    

    app = QApplication(sys.argv)
    gui = GuiApp(hand_type=hand_type,hand_joint=hand_joint,hz=hz)
    gui.node = node
    time.sleep(1)
    gui.loop_pub()
    gui.show()

    sys.exit(app.exec_())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
