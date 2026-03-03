#!/home/rem/rtimulib-env/bin/python3

import sys
import os
import signal
import subprocess
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from PySide6.QtWidgets import (
    QApplication, QWidget, QLabel, QVBoxLayout, QHBoxLayout, QGridLayout,
    QGroupBox, QDoubleSpinBox, QPushButton, QScrollArea
)
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtCore import QTimer, Qt

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure


class MotorPlotWidget(QWidget):
    """Widget pour afficher un graphe individuel d'un moteur"""
    def __init__(self, motor_number, max_len=100):
        super().__init__()
        self.motor_number = motor_number
        self.max_len = max_len
        self.feedback_data = deque([0]*max_len, maxlen=max_len)
        self.current_consigne = 0.0

        layout = QVBoxLayout()
        self.setLayout(layout)

        self.fig = Figure(figsize=(2, 1.5))
        self.canvas = FigureCanvas(self.fig)
        layout.addWidget(self.canvas)

        self.ax = self.fig.add_subplot(111)
        self.ax.set_title(f"M{motor_number}", fontsize=8)
        self.ax.set_ylabel("L(m)", fontsize=6)
        self.ax.set_xlabel("T", fontsize=6)
        self.ax.tick_params(labelsize=5)

        self.line_consigne = self.ax.axhline(y=0, color='blue', linewidth=1.5,
                                             label='Cons', linestyle='--')
        self.line_feedback = self.ax.plot(range(self.max_len), self.feedback_data,
                                          label='Feed', color='red', linewidth=1)[0]
        self.ax.legend(fontsize=5, loc='upper right')
        self.ax.set_ylim(0, 0.5)
        self.ax.grid(True, alpha=0.3)

    def update_plot(self, consigne_value, feedback_value):
        if consigne_value != self.current_consigne:
            self.current_consigne = consigne_value
            self.line_consigne.set_ydata([consigne_value, consigne_value])

        self.feedback_data.append(feedback_value)
        self.line_feedback.set_ydata(self.feedback_data)

        all_values = list(self.feedback_data) + [self.current_consigne]
        if all_values:
            min_val = min(all_values)
            max_val = max(all_values)
            margin = (max_val - min_val) * 0.1 if max_val > min_val else 0.1
            self.ax.set_ylim(min_val - margin, max_val + margin)

        self.canvas.draw()


class InterfaceGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Stewart Platform")
        self.setMinimumSize(800, 480)

        self.setStyleSheet("""
            QWidget { background-color: #f4f6f9; font-family: Arial; font-size: 11px; }
            QGroupBox { font-weight: bold; font-size: 12px; border: 1px solid #4a90e2; 
                       border-radius: 4px; margin-top: 6px; background-color: #ffffff; padding: 4px; }
            QLabel { font-size: 11px; padding: 2px; }
            QDoubleSpinBox { font-size: 10px; padding: 2px; max-height: 24px; }
            QPushButton { padding: 5px; font-size: 10px; max-height: 28px; border-radius: 3px; }
            QPushButton#start_button { background-color: #4CAF50; color: white; }
            QPushButton#stop_button { background-color: #f44336; color: white; }
        """)

        # Publishers ROS2
        self.pos_pub = None
        self.ori_pub = None

        main_layout = QHBoxLayout()
        main_layout.setContentsMargins(4, 4, 4, 4)
        main_layout.setSpacing(4)

        # ========== COLONNE GAUCHE ==========
        left_layout = QVBoxLayout()
        left_layout.setSpacing(4)

        # --- Saisie manuelle compacte ---
        manual_group = QGroupBox("⚙ Contrôle")
        manual_layout = QGridLayout()
        manual_layout.setSpacing(2)

        self.input_x = QDoubleSpinBox(); self.input_x.setRange(-1,1); self.input_x.setDecimals(3); self.input_x.setSingleStep(0.01)
        self.input_y = QDoubleSpinBox(); self.input_y.setRange(-1,1); self.input_y.setDecimals(3); self.input_y.setSingleStep(0.01)
        self.input_z = QDoubleSpinBox(); self.input_z.setRange(-1,1); self.input_z.setDecimals(3); self.input_z.setSingleStep(0.01)
        self.input_roll = QDoubleSpinBox(); self.input_roll.setRange(-180,180); self.input_roll.setSingleStep(1)
        self.input_pitch = QDoubleSpinBox(); self.input_pitch.setRange(-180,180); self.input_pitch.setSingleStep(1)
        self.input_yaw = QDoubleSpinBox(); self.input_yaw.setRange(-180,180); self.input_yaw.setSingleStep(1)

        manual_layout.addWidget(QLabel("X(cm)"), 0, 0); manual_layout.addWidget(self.input_x, 0, 1)
        manual_layout.addWidget(QLabel("Y(cm)"), 1, 0); manual_layout.addWidget(self.input_y, 1, 1)
        manual_layout.addWidget(QLabel("Z(cm)"), 2, 0); manual_layout.addWidget(self.input_z, 2, 1)
        manual_layout.addWidget(QLabel("Roll(°)"), 0, 2); manual_layout.addWidget(self.input_roll, 0, 3)
        manual_layout.addWidget(QLabel("Pitch(°)"), 1, 2); manual_layout.addWidget(self.input_pitch, 1, 3)
        manual_layout.addWidget(QLabel("Yaw(°)"), 2, 2); manual_layout.addWidget(self.input_yaw, 2, 3)

        self.button_update_manual = QPushButton("Appliquer")
        self.button_update_manual.setObjectName("start_button")
        self.button_update_manual.clicked.connect(self.send_manual_pose)
        manual_layout.addWidget(self.button_update_manual, 3, 0, 1, 4)

        manual_group.setLayout(manual_layout)
        left_layout.addWidget(manual_group)

        # --- Boutons compacts (2x2 + Stop) ---
        btn_group = QGroupBox("🎮 Actions")
        btn_layout = QGridLayout()
        btn_layout.setSpacing(2)

        self.button_start_acq = QPushButton("Acquisition")
        self.button_start_acq.setObjectName("start_button"); self.button_start_acq.clicked.connect(self.start_acquisition)
        self.button_start_att = QPushButton("Automatique")
        self.button_start_att.setObjectName("start_button"); self.button_start_att.clicked.connect(self.start_attelage)
        self.button_manual = QPushButton("Manuel")
        self.button_manual.setObjectName("start_button"); self.button_manual.clicked.connect(self.start_manual_node)
        self.button_uart = QPushButton("État initial")
        self.button_uart.setObjectName("start_button"); self.button_uart.clicked.connect(self.start_uart_node)
        self.button_stop = QPushButton("⏹ ARRÊTER")
        self.button_stop.setObjectName("stop_button"); self.button_stop.clicked.connect(self.stop_ros_launch)

        btn_layout.addWidget(self.button_start_acq, 0, 0)
        btn_layout.addWidget(self.button_start_att, 0, 1)
        btn_layout.addWidget(self.button_manual, 1, 0)
        btn_layout.addWidget(self.button_uart, 1, 1)
        btn_layout.addWidget(self.button_stop, 2, 0, 1, 2)

        btn_group.setLayout(btn_layout)
        left_layout.addWidget(btn_group)

        # --- Vidéo ArUco compacte ---
        video_group = QGroupBox("📹 Caméra")
        video_layout = QVBoxLayout()
        video_layout.setContentsMargins(2, 2, 2, 2)
        self.video_label = QLabel()
        self.video_label.setFixedSize(320, 240)
        self.video_label.setStyleSheet("border: 1px solid gray; background-color: black;")
        self.video_label.setScaledContents(True)
        video_layout.addWidget(self.video_label)
        video_group.setLayout(video_layout)
        left_layout.addWidget(video_group)

        left_layout.addStretch()
        main_layout.addLayout(left_layout, 35)

        # ========== COLONNE DROITE ==========
        right_layout = QVBoxLayout()
        right_layout.setSpacing(4)

        # --- ArUco Position & Orientation ---
        aruco_group = QGroupBox("🎯 ArUco")
        aruco_layout = QVBoxLayout()
        aruco_layout.setSpacing(4)
        self.label_aruco = QLabel("Position: --")
        self.label_aruco.setStyleSheet("font-size: 12px;")
        self.label_aruco_ori = QLabel("Orientation: --")
        self.label_aruco_ori.setStyleSheet("font-size: 11px;")
        aruco_layout.addWidget(self.label_aruco)
        aruco_layout.addWidget(self.label_aruco_ori)
        aruco_group.setLayout(aruco_layout)
        right_layout.addWidget(aruco_group)

        # --- IMU ---
        imu_group = QGroupBox("📐 IMU")
        imu_layout = QVBoxLayout()
        imu_layout.setSpacing(2)
        self.label_imu = QLabel("IMU: --")
        self.label_imu.setStyleSheet("font-size: 11px;")
        imu_layout.addWidget(self.label_imu)
        imu_group.setLayout(imu_layout)
        right_layout.addWidget(imu_group)

        # --- FUSION ---
        fusion_group = QGroupBox("🧩 Fusion")
        fusion_layout = QVBoxLayout()
        fusion_layout.setSpacing(2)
        self.label_fusion = QLabel("Fusion: --")
        self.label_fusion.setStyleSheet("font-size: 11px;")
        fusion_layout.addWidget(self.label_fusion)
        fusion_group.setLayout(fusion_layout)
        right_layout.addWidget(fusion_group)


        # --- Consignes & Feedback ---
        data_group = QGroupBox("📊 Moteurs")
        data_layout = QVBoxLayout()
        data_layout.setSpacing(2)
        self.label_verins = QLabel("Consignes: --")
        self.label_feedback = QLabel("Feedback: --")
        data_layout.addWidget(self.label_verins)
        data_layout.addWidget(self.label_feedback)
        data_group.setLayout(data_layout)
        right_layout.addWidget(data_group)

        # --- Graphes moteurs (3x2 grid, compacts) ---
        plots_group = QGroupBox("📈 Graphes")
        plots_layout = QGridLayout()
        plots_layout.setSpacing(2)
        self.motor_plots = []
        for i in range(6):
            plot = MotorPlotWidget(i+1)
            self.motor_plots.append(plot)
            row, col = divmod(i, 3)
            plots_layout.addWidget(plot, row, col)
        plots_group.setLayout(plots_layout)
        right_layout.addWidget(plots_group)

        right_layout.addStretch()
        main_layout.addLayout(right_layout, 65)

        self.setLayout(main_layout)

        # ROS2 processes
        self.acquisition_process = None
        self.fusion_process = None   # ✅ NOUVEAU
        self.attelage_process = None
        self.uart_process = None
        self.manual_process = None

        self.last_consigne = [0]*6

    # --- Fonctions boutons ---
    def start_acquisition(self):
        if self.acquisition_process and self.acquisition_process.poll() is None: return
        self.acquisition_process = subprocess.Popen(["ros2","launch","stewart_control","stewart_ordered_launch.py"], preexec_fn=os.setsid)

        # ✅ Lance aussi la fusion automatiquement
        if not self._is_running(self.fusion_process):
            self.fusion_process = self._start_process(
                ["ros2", "run", "stewart_control", "fusion_node"]
            )

    def start_attelage(self):
        if self.attelage_process and self.attelage_process.poll() is None: return
        self.attelage_process = subprocess.Popen(["ros2","launch","stewart_control","stewart.launch.py"], preexec_fn=os.setsid)

    def start_manual_node(self):
        if self.manual_process and self.manual_process.poll() is None: return
        self.manual_process = subprocess.Popen(["ros2","launch","stewart_control","manual_launch.py"], preexec_fn=os.setsid)

    def start_uart_node(self):
        if self.uart_process and self.uart_process.poll() is None: return
        self.uart_process = subprocess.Popen(["ros2","launch","stewart_control","launch_posHome.py"], preexec_fn=os.setsid)

    def stop_ros_launch(self):
        for proc in [self.fusion_process,self.acquisition_process,self.attelage_process,self.uart_process,self.manual_process]:
            if proc and proc.poll() is None:
                try: os.killpg(os.getpgid(proc.pid), signal.SIGINT); proc.wait(timeout=5)
                except: pass
                finally: proc.kill()
        subprocess.call(["pkill","-f","ros2 launch"])
        self.fusion_process = self.acquisition_process = self.attelage_process = self.uart_process = self.manual_process = None

    def send_manual_pose(self):
        if not self.manual_process: return
        pos_msg = Float32MultiArray(); pos_msg.data = [self.input_x.value(),self.input_y.value(),self.input_z.value()]
        self.pos_pub.publish(pos_msg)
        ori_msg = Float32MultiArray(); ori_msg.data = [self.input_roll.value(),self.input_pitch.value(),self.input_yaw.value()]
        self.ori_pub.publish(ori_msg)

    # --- Callbacks ROS ---
    def imu_callback(self, msg):
        if len(msg.data)>=3: 
            roll,pitch,yaw=[round(v,2) for v in msg.data[:3]]
            self.label_imu.setText(f"Roll={roll} Pitch={pitch} Yaw={yaw}")

    def fusion_callback(self, msg):
        if len(msg.data) >= 3:
            roll, pitch, yaw = [round(v, 2) for v in msg.data[:3]]
            self.label_fusion.setText(f"Roll={roll} Pitch={pitch} Yaw={yaw}")


    def verins_callback(self,msg):
        if len(msg.data)>=6:
            vals=[round(v,3) for v in msg.data[:6]]
            self.label_verins.setText("C: " + ", ".join([f"V{i+1}:{v}" for i,v in enumerate(vals)]))
            self.last_consigne=vals

    def feedback_callback(self,msg):
        if len(msg.data)>=6:
            vals=[round(v,3) for v in msg.data[:6]]
            self.label_feedback.setText("F: " + ", ".join([f"V{i+1}:{v}" for i,v in enumerate(vals)]))
            for i in range(6): self.motor_plots[i].update_plot(self.last_consigne[i],vals[i])

    def aruco_callback(self,msg):
        if len(msg.data)>=3:
            x,y,z=[round(v,3) for v in msg.data[:3]]
            self.label_aruco.setText(f"Pos: X={x} Y={y} Z={z}")

    def aruco_orientation_callback(self,msg):
        if len(msg.data)>=3:
            roll,pitch,yaw=[round(v,1) for v in msg.data[:3]]
            self.label_aruco_ori.setText(f"Ori: Roll={roll}° Pitch={pitch}° Yaw={yaw}°")

    def image_callback(self,msg):
        try:
            cv_img = CvBridge().imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h,w,ch=cv_img.shape; bytes_per_line=ch*w
            qt_img=QImage(cv_img.data,w,h,bytes_per_line,QImage.Format_BGR888)
            self.video_label.setPixmap(QPixmap.fromImage(qt_img))
        except Exception as e:
            print(f"Erreur conversion image: {e}")


class InterfaceNode(Node):
    def __init__(self, gui):
        super().__init__('interface_node_gui')
        self.gui=gui
        self.bridge=CvBridge()
        self.create_subscription(Float32MultiArray,'imu_error',self.gui.imu_callback,10)
        self.create_subscription(Float32MultiArray, 'F_orientation', self.gui.fusion_callback, 10)
        self.create_subscription(Float32MultiArray,'stewart/longueurs',self.gui.verins_callback,10)
        self.create_subscription(Float32MultiArray,'feedback_motors',self.gui.feedback_callback,10)
        self.create_subscription(Float32MultiArray,'aruco_position',self.gui.aruco_callback,10)
        self.create_subscription(Image,'camera/image_raw',self.gui.image_callback,10)
        self.create_subscription(Float32MultiArray,'aruco_orientation',self.gui.aruco_orientation_callback,10)
        self.gui.pos_pub=self.create_publisher(Float32MultiArray,'manual_position',10)
        self.gui.ori_pub=self.create_publisher(Float32MultiArray,'manual_orientation',10)
        self.gui.node=self


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    gui = InterfaceGUI()
    node = InterfaceNode(gui)
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node,timeout_sec=0.01))
    timer.start(30)
    gui.show()
    app.exec()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
