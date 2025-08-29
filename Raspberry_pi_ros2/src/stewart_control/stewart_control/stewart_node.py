#!/home/rem/rtimulib-env/bin/python3

import numpy as np
import serial
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from stewart_control.inv_kinematics import StewartPlatform


class StewartNode(Node):
    def __init__(self):
        super().__init__('stewart_node')

        # Publisher pour les longueurs des vérins
        self.publisher_ = self.create_publisher(Float32MultiArray, 'stewart/longueurs', 10)

        # Subscriber pour les erreur d'orientation IMU
        self.imu_subscription = self.create_subscription(
            Float32MultiArray, 'imu_error', self.imu_callback, 10
        )

        # Subscriber pour la position ArUco
        self.pos_subscription = self.create_subscription(
            Float32MultiArray, 'aruco_position', self.position_callback, 10
        )

        # UART vers Arduino
        self.ser = serial.Serial("/dev/ttyAMA0", 9600, timeout=1)
        time.sleep(2)

        # Plateforme Stewart
        radious_platform, radious_base = 0.2, 0.2
        half_angle_platform, half_angle_base = 12, 12
        self.platform = StewartPlatform(radious_base, radious_platform, half_angle_base, half_angle_platform)

        # Longueurs repos
        self.L0 = np.array([25, 25, 25, 25, 25, 25])

        # Dernière position ArUco
        self.position = np.array([0.0, 0.0, 0.0])

        # Flag pour envoyer consignes continuellement ou une seule fois
        self.continuous = True

    def position_callback(self, msg):
        if len(msg.data) >= 3:
            self.position = np.array(msg.data)  # X, Y, Z en mètres

    def imu_callback(self, msg):
        if len(msg.data) < 3:
            self.get_logger().warning("Erreur IMU: données insuffisantes")
            return

        rotation = np.array(msg.data)  # Roll, Pitch, Yaw en degrés
        trans = self.position          # Position ArUco en mètres

        # Calcul des longueurs
        longueurs_m = self.platform.solve(trans, rotation)
        longueurs_cm = longueurs_m * 100
        deplacement = longueurs_cm - self.L0

        # Envoi UART
        consigne = ",".join([f"{d:.2f}" for d in deplacement])
        self.ser.write((consigne + "\n").encode())
        self.get_logger().info(f"Consigne envoyée : {consigne}")

        # Publication ROS2
        out_msg = Float32MultiArray()
        out_msg.data = deplacement.tolist()
        self.publisher_.publish(out_msg)

        # Lecture retour Arduino
        while self.ser.in_waiting:
            self.get_logger().info("🟢 Arduino: " + self.ser.readline().decode(errors="ignore").strip())

        # Si on n’est pas en mode continu, arrêter après la première consigne
        if not self.continuous:
            self.get_logger().info("Consigne envoyée une fois, arrêt du callback.")
            self.imu_subscription.unregister()

def main(args=None):
    rclpy.init(args=args)
    node = StewartNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
