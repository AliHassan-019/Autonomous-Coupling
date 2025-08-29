#!/home/rem/rtimulib-env/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import cv2
import cv2.aruco as aruco
import numpy as np

class ArucoPublisher(Node):
    def __init__(self):
        super().__init__('aruco_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'aruco_position', 10)

        # Vérifie que la caméra est bien ouverte
        self.cap = cv2.VideoCapture(0) #des fois c(1)
        if not self.cap.isOpened():
            self.get_logger().error("Impossible d'ouvrir la caméra ! Vérifie l'index ou le périphérique.")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1600)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1200)

        # Nouveau style OpenCV pour ArUco
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters_create()

        # Paramètres caméra (fx, fy, cx, cy)
        fx = fy = 1279.6251545786083
        cx, cy = 1600 / 2, 1200 / 2
        self.camera_matrix = np.array([[fx, 0, cx],
                                       [0, fy, cy],
                                       [0, 0, 1]], dtype=np.float64)
        self.dist_coeffs = np.zeros((5,), dtype=np.float64)
        self.marker_size = 0.1  # en mètres

        # Timer ROS pour publication périodique
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Impossible de lire la frame de la caméra")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None and len(ids) > 0:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_size,
                                                              self.camera_matrix, self.dist_coeffs)
            tvec = tvecs[0][0]

            # Transformation pour repère robot
            x_robot = tvec[2]
            y_robot = -tvec[0]
            z_robot = -tvec[1]
            trans_robot = [x_robot, y_robot, z_robot]

            # Publication ROS2
            msg = Float32MultiArray()
            msg.data = trans_robot
            self.publisher_.publish(msg)
            self.get_logger().info(f"Position ArUco publiée: {trans_robot}")

def main(args=None):
    rclpy.init(args=args)
    node = ArucoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
