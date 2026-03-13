#!/home/rem/rtimulib-env/bin/python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import cv2.aruco as aruco
import numpy as np
import math
import os
import time
from ament_index_python.packages import get_package_share_directory

class ArucoRelativePose(Node):
    def __init__(self):
        super().__init__('aruco_relative_pose')

        # -------- Publishers --------
        self.pub_pos = self.create_publisher(Float32MultiArray, 'aruco_position', 10)
        self.pub_ori = self.create_publisher(Vector3Stamped, 'aruco_orientation', 10)
        self.pub_img = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()

        # -------- Caméra --------
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Caméra introuvable (/dev/video0). Vérifie le câble ou les permissions.")
            return
        else:
            w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            fps = self.cap.get(cv2.CAP_PROP_FPS)
            self.get_logger().info(f"Caméra OK: {w}x{h} @ {fps} fps")

        # -------- ArUco --------
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters_create()

        # -------- Calibrations (intr & extr) --------
        intr_path, extr_path = self._resolve_calib_paths()
        self.get_logger().info(f"Calibration intrinsèque: {intr_path}")
        self.get_logger().info(f"Calibration extrinsèque: {extr_path}")

        try:
            intr = np.load(intr_path)
            self.mtx = intr['mtx']
            self.dist = intr['dist']
        except Exception as e:
            self.get_logger().error(f"Erreur ouverture {intr_path}: {e}")
            return

        try:
            extr = np.load(extr_path)
            self.rvec_ext = extr["rvec"].reshape(3,1)
            self.tvec_ext = extr["tvec"].reshape(3,1)
            self.R_ext, _ = cv2.Rodrigues(self.rvec_ext)
        except Exception as e:
            self.get_logger().error(f"Erreur ouverture {extr_path}: {e}")
            return

        self.marker_size = 0.022  # m
        self.fixed_id  = 34
        self.mobile_id = 28

        self.frame_count = 0
        self.t0 = time.time()

        # 10 Hz
        self.timer = self.create_timer(0.1, self.loop)
        self.get_logger().info("aruco_node prêt (détection à ~10 Hz).")

    # Essayez d'abord le share du package; si absent, bascule en chemins relatifs au script.
    def _resolve_calib_paths(self):
        try:
            pkg = get_package_share_directory('stewart_control')
        except Exception as e:
            pkg = None
            self.get_logger().warn(f"get_package_share_directory a échoué: {e}")

        candidates = []
        if pkg:
            candidates.append((
                os.path.join(pkg, "calib_int.npz"),
                os.path.join(pkg, "calib_ext3.npz")
            ))

        # fallback: dans le même dossier que ce script
        base = os.path.dirname(__file__)
        candidates.append((
            os.path.join(base, "calib_int.npz"),
            os.path.join(base, "calib_ext3.npz")
        ))

        for intr, extr in candidates:
            if os.path.isfile(intr) and os.path.isfile(extr):
                return intr, extr

        # Si rien trouvé, renvoie le premier pair (pour log d'erreur clair)
        return candidates[0]

    def rvec_to_euler(self, rvec):
        R, _ = cv2.Rodrigues(rvec)
        sy = math.sqrt(R[0,0]**2 + R[1,0]**2)
        singular = sy < 1e-6
        if not singular:
            roll = math.atan2(R[2,1], R[2,2])
            pitch = math.atan2(-R[2,0], sy)
            yaw = math.atan2(R[1,0], R[0,0])
        else:
            roll = math.atan2(-R[1,2], R[1,1])
            pitch = math.atan2(-R[2,0], sy)
            yaw = 0.0
        return [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]

    def loop(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("frame grab raté")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        camera_ready = False
        detected = []

        if ids is not None:
            ids_flat = ids.flatten().tolist()
            detected = ids_flat
            aruco.drawDetectedMarkers(frame, corners, ids)

            # --- Marker FIXE ---
            if self.fixed_id in ids_flat:
                idx = ids_flat.index(self.fixed_id)
                rvec_f, tvec_f, _ = aruco.estimatePoseSingleMarkers(
                    [corners[idx]], self.marker_size, self.mtx, self.dist
                )
                rvec_f = rvec_f[0].reshape(3,1)
                tvec_f = tvec_f[0].reshape(3,1)

                R_f, _      = cv2.Rodrigues(rvec_f)
                R_cam_world = R_f.T
                t_cam_world = -R_f.T @ tvec_f

                camera_ready = True
                aruco.drawAxis(frame, self.mtx, self.dist, rvec_f, tvec_f, 0.05)

            # --- Marker MOBILE ---
            if self.mobile_id in ids_flat and camera_ready:
                idx = ids_flat.index(self.mobile_id)
                rvec_m, tvec_m, _ = aruco.estimatePoseSingleMarkers(
                    [corners[idx]], self.marker_size, self.mtx, self.dist
                )
                rvec_m = rvec_m[0].reshape(3,1)
                tvec_m = tvec_m[0].reshape(3,1)

                R_m, _ = cv2.Rodrigues(rvec_m)
                t_m_world = R_cam_world @ tvec_m + t_cam_world
                R_m_world = R_cam_world @ R_m

                t_mobile = t_m_world.flatten()
                roll, pitch, yaw = self.rvec_to_euler(cv2.Rodrigues(R_m_world)[0])

                # position
                msg_pos = Float32MultiArray()
                msg_pos.data = [float(t_mobile[0]), float(t_mobile[1]), float(t_mobile[2])]
                self.pub_pos.publish(msg_pos)

                # orientation (Vector3Stamped)
                msg_ori = Vector3Stamped()
                msg_ori.header.stamp = self.get_clock().now().to_msg()
                msg_ori.header.frame_id = 'aruco'
                msg_ori.vector.x = float(roll)
                msg_ori.vector.y = float(pitch)
                msg_ori.vector.z = float(yaw)
                self.pub_ori.publish(msg_ori)

                cv2.putText(frame, f"R={roll:.1f} P={pitch:.1f} Y={yaw:.1f}",
                            (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,100), 2)

        # Stats (toutes les ~30 frames)
        self.frame_count += 1
        if self.frame_count % 30 == 0:
            dt = time.time() - self.t0
            fps = self.frame_count / max(dt, 1e-6)
            self.get_logger().info(f"fps~{fps:.1f} | ids={detected}")

        # Publish image
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.pub_img.publish(img_msg)

    def destroy_node(self):
        try:
            if self.cap and self.cap.isOpened():
                self.cap.release()
        finally:
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

if __name__ == '__main__':
    main()
