#!/home/rem/rtimulib-env/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import cv2.aruco as aruco
import numpy as np
import math
import os
from ament_index_python.packages import get_package_share_directory


class ArucoRelativePose(Node):

    def __init__(self):
        super().__init__("aruco_relative_pose")

        # ------ Publishers ------
        # Tes anciens noms:
        self.pub_pos = self.create_publisher(Float32MultiArray, "aruco_position", 10)
        self.pub_ori = self.create_publisher(Float32MultiArray, "aruco_orientation", 10)
        self.pub_img = self.create_publisher(Image, "camera/image_raw", 10)

        self.bridge = CvBridge()

        # ------ caméra ------
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Erreur : impossible d'ouvrir la caméra.")
            return

        # ------ ArUco ------
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters_create()

        # ------ Charger calibration intrinsèque + extrinsèque ------
        pkg = get_package_share_directory("stewart_control")

        intr_path = os.path.join(pkg, "calib_int.npz")
        extr_path = os.path.join(pkg, "calib_ext3.npz")

        intr = np.load(intr_path)
        self.mtx = intr["mtx"]
        self.dist = intr["dist"]
        self.marker_size = 0.022  # m

        extr = np.load(extr_path)
        self.rvec_ext = extr["rvec"].reshape(3, 1)
        self.tvec_ext = extr["tvec"].reshape(3, 1)
        self.R_ext, _ = cv2.Rodrigues(self.rvec_ext)

        # IDs
        self.fixed_id = 34  # ID du repère fixe
        self.mobile_id = 28  # ID mobile

        # Timer ROS2 (10 Hz)
        self.timer = self.create_timer(0.1, self.loop)

    def rvec_to_euler(self, rvec):
        R, _ = cv2.Rodrigues(rvec)
        sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
        singular = sy < 1e-6
        if not singular:
            roll = math.atan2(R[2, 1], R[2, 2])
            pitch = math.atan2(-R[2, 0], sy)
            yaw = math.atan2(R[1, 0], R[0, 0])
        else:
            roll = math.atan2(-R[1, 2], R[1, 1])
            pitch = math.atan2(-R[2, 0], sy)
            yaw = 0
        return [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]

    def loop(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.parameters
        )

        camera_ready = False

        if ids is not None:

            ids_flat = ids.flatten()
            aruco.drawDetectedMarkers(frame, corners, ids)

            if self.fixed_id in ids_flat:
                idx = ids_flat.tolist().index(self.fixed_id)

                rvec_f, tvec_f, _ = aruco.estimatePoseSingleMarkers(
                    [corners[idx]], self.marker_size, self.mtx, self.dist
                )
                rvec_f = rvec_f[0].reshape(3, 1)
                tvec_f = tvec_f[0].reshape(3, 1)

                R_f, _ = cv2.Rodrigues(rvec_f)
                R_cam_world = R_f.T
                t_cam_world = -R_f.T @ tvec_f

                camera_ready = True

                aruco.drawAxis(frame, self.mtx, self.dist, rvec_f, tvec_f, 0.05)

            if self.mobile_id in ids_flat and camera_ready:

                idx = ids_flat.tolist().index(self.mobile_id)

                rvec_m, tvec_m, _ = aruco.estimatePoseSingleMarkers(
                    [corners[idx]], self.marker_size, self.mtx, self.dist
                )
                rvec_m = rvec_m[0].reshape(3, 1)
                tvec_m = tvec_m[0].reshape(3, 1)

                R_m, _ = cv2.Rodrigues(rvec_m)
                t_m_world = R_cam_world @ tvec_m + t_cam_world
                R_m_world = R_cam_world @ R_m

                t_mobile_in_fixed = t_m_world
                t_mobile_cm = t_mobile_in_fixed.flatten()  # * 100.0

                roll, pitch, yaw = self.rvec_to_euler(cv2.Rodrigues(R_m_world)[0])

                msg_pos = Float32MultiArray()
                msg_pos.data = [
                    float(t_mobile_cm[0]),
                    float(t_mobile_cm[1]),
                    float(t_mobile_cm[2]),
                ]
                self.pub_pos.publish(msg_pos)

                msg_ori = Float32MultiArray()
                msg_ori.data = [float(roll), float(pitch), float(yaw)]
                self.pub_ori.publish(msg_ori)

                text_pos = (
                    f"Mobile: X={t_mobile_cm[0]:.1f} Y={t_mobile_cm[1]:.1f} "
                    f"Z={t_mobile_cm[2]:.1f} cm"
                )
                cv2.putText(
                    frame,
                    text_pos,
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2,
                )
                cv2.putText(
                    frame,
                    f"R={roll:.1f} P={pitch:.1f} Y={yaw:.1f}",
                    (20, 70),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 100),
                    2,
                )

        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.pub_img.publish(img_msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoRelativePose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
