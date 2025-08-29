#!/home/rem/rtimulib-env/bin/python3

import sys
import os
import signal
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from PySide6.QtWidgets import (
    QApplication, QWidget, QLabel, QVBoxLayout, QHBoxLayout,
    QGroupBox, QGridLayout, QFrame, QPushButton
)
from PySide6.QtCore import QTimer, Qt


class InterfaceGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Interface Stewart Platform")
        self.setMinimumSize(700, 400)  # un peu plus large pour les boutons

        # -------- STYLE GLOBAL --------
        self.setStyleSheet("""
            QWidget {
                background-color: #f4f6f9;
                font-family: Segoe UI, Arial;
                font-size: 14px;
            }
            QGroupBox {
                font-weight: bold;
                font-size: 15px;
                border: 2px solid #4a90e2;
                border-radius: 8px;
                margin-top: 10px;
                background-color: #ffffff;
                padding: 8px;
            }
            QLabel {
                font-size: 14px;
                padding: 4px;
            }
            QPushButton {
                padding: 8px;
                font-size: 14px;
            }
            QPushButton#start_button {
                background-color: #4CAF50;
                color: white;
            }
            QPushButton#stop_button {
                background-color: #f44336;
                color: white;
            }
        """)

        main_layout = QHBoxLayout()  # layout principal horizontal

        # ----- Colonne gauche : affichage des données -----
        left_layout = QVBoxLayout()
        left_layout.setSpacing(15)
        left_layout.setContentsMargins(15, 15, 15, 15)

        # Section Erreurs IMU
        imu_group = QGroupBox("Erreur IMU (X, Y, Z)")
        imu_layout = QGridLayout()
        self.label_imu = QLabel("Erreur: --")
        self.label_imu.setAlignment(Qt.AlignCenter)
        imu_layout.addWidget(self.label_imu, 0, 0)
        imu_group.setLayout(imu_layout)

        # Section Vérins
        verins_group = QGroupBox("⚙️ Longueurs des Vérins")
        verins_layout = QGridLayout()
        self.label_verins = QLabel("Vérins: --")
        self.label_verins.setAlignment(Qt.AlignCenter)
        verins_layout.addWidget(self.label_verins, 0, 0)
        verins_group.setLayout(verins_layout)

        # Section Position ArUco
        aruco_group = QGroupBox("🎯 Position ArUco (X, Y, Z)")
        aruco_layout = QGridLayout()
        self.label_aruco = QLabel("Position: --")
        self.label_aruco.setAlignment(Qt.AlignCenter)
        aruco_layout.addWidget(self.label_aruco, 0, 0)
        aruco_group.setLayout(aruco_layout)

        # Ajouter au layout gauche
        left_layout.addWidget(imu_group)
        left_layout.addWidget(verins_group)
        left_layout.addWidget(aruco_group)

        # Ligne séparatrice
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        left_layout.addWidget(line)

        # Footer
        footer = QLabel("Projet REM par ABMI")
        footer.setAlignment(Qt.AlignCenter)
        footer.setStyleSheet("color: gray; font-size: 12px;")
        left_layout.addWidget(footer)

        main_layout.addLayout(left_layout)  # ajout colonne gauche

        # ----- Colonne droite : boutons Start/Stop -----
        right_layout = QVBoxLayout()
        right_layout.setSpacing(20)
        right_layout.setContentsMargins(10, 20, 20, 20)

        self.button_start = QPushButton("Commencer l'attelage")
        self.button_start.setObjectName("start_button")
        self.button_start.clicked.connect(self.start_ros_launch)
        right_layout.addWidget(self.button_start)

        self.button_stop = QPushButton("Arrêter l'attelage")
        self.button_stop.setObjectName("stop_button")
        self.button_stop.clicked.connect(self.stop_ros_launch)
        right_layout.addWidget(self.button_stop)

        right_layout.addStretch()  # pousse les boutons vers le haut
        main_layout.addLayout(right_layout)  # ajout colonne droite

        self.setLayout(main_layout)
        self.process = None  # pour stocker le process ROS2

    # ----- Fonctions Start / Stop -----
    def start_ros_launch(self):
        if self.process and self.process.poll() is None:
            print("Le process ROS2 est déjà en cours")
            return
        self.process = subprocess.Popen(
            ["ros2", "launch", "stewart_control", "stewart_ordered_launch.py"],
            preexec_fn=os.setsid
        )
        print("ROS2 launch démarré")

    def stop_ros_launch(self):
        if self.process and self.process.poll() is None:
            os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
            print("ROS2 launch arrêté")
        else:
            print("Aucun process ROS2 à arrêter")


class InterfaceNode(Node):
    def __init__(self, gui):
        super().__init__('interface_node_gui')
        self.gui = gui

        self.create_subscription(Float32MultiArray, 'imu_error', self.imu_callback, 10)
        self.create_subscription(Float32MultiArray, 'stewart/longueurs', self.verins_callback, 10)
        self.create_subscription(Float32MultiArray, 'aruco_position', self.aruco_callback, 10)

    def imu_callback(self, msg):
        if len(msg.data) >= 3:
            roll, pitch, yaw = [round(val, 3) for val in msg.data[:3]]
            self.gui.label_imu.setText(f"Roll = {roll}, Pitch = {pitch}, Yaw = {yaw}")
        else:
            self.gui.label_imu.setText("Erreur IMU: données insuffisantes")

    def verins_callback(self, msg):
        if len(msg.data) >= 6:
            verins = [round(val, 3) for val in msg.data[:6]]
            verins_text = ", ".join([f"V{i+1} = {v}" for i, v in enumerate(verins)])
            self.gui.label_verins.setText(verins_text)
        else:
            self.gui.label_verins.setText("Vérins: données insuffisantes")

    def aruco_callback(self, msg):
        if len(msg.data) >= 3:
            x, y, z = [round(val, 3) for val in msg.data[:3]]
            self.gui.label_aruco.setText(f"X = {x}, Y = {y}, Z = {z} m")
        else:
            self.gui.label_aruco.setText("Position ArUco: données insuffisantes")


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    gui = InterfaceGUI()
    node = InterfaceNode(gui)

    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.01))
    timer.start(10)

    gui.show()
    app.exec()

    if gui.process and gui.process.poll() is None:
        os.killpg(os.getpgid(gui.process.pid), signal.SIGINT)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
