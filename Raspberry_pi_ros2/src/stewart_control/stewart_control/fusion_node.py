#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from stewart_control.fusion_utils import wrap_deg, Kalman1D

# ============================================================
# FUSION NODE
# ============================================================


class FusionNode(Node):

    def __init__(self):
        super().__init__("fusion_node")

        # Subscribers
        self.sub_imu = self.create_subscription(
            Float32MultiArray, "imu_error", self.imu_callback, 10
        )

        self.sub_cam = self.create_subscription(
            Float32MultiArray, "aruco_orientation", self.cam_callback, 10
        )

        # Publisher
        self.pub_fusion = self.create_publisher(Float32MultiArray, "F_orientation", 10)

        # Kalman filters
        self.kf_roll = Kalman1D(q=0.02, r=1.0)
        self.kf_pitch = Kalman1D(q=0.02, r=1.0)
        self.kf_yaw = Kalman1D(q=0.05, r=2.0)

        self.last_imu = None
        self.last_cam = None

        self.yaw_moy = 0.0
        self.ALPHA_YM = 0.05

        self.get_logger().info("Fusion node démarré ✅")

    # ============================================================
    # IMU callback
    # ============================================================

    def imu_callback(self, msg):
        self.last_imu = msg.data
        self.compute_fusion()

    # ============================================================
    # CAM callback
    # ============================================================

    def cam_callback(self, msg):
        self.last_cam = msg.data
        self.compute_fusion()

    # ============================================================
    # Fusion
    # ============================================================

    def compute_fusion(self):

        if self.last_imu is None:
            return

        r_imu, p_imu, y_imu = self.last_imu

        # Correction yaw IMU dynamique
        self.yaw_moy = (1.0 - self.ALPHA_YM) * self.yaw_moy + self.ALPHA_YM * y_imu
        y_corr = wrap_deg(y_imu - self.yaw_moy)

        # Predict (IMU)
        self.kf_roll.predict(r_imu)
        self.kf_pitch.predict(p_imu)
        self.kf_yaw.predict(y_corr)

        # Update si caméra dispo
        if self.last_cam is not None:

            r_cam, p_cam, y_cam = self.last_cam

            # Permutation CAM -> IMU
            r_cam_imu = p_cam
            p_cam_imu = y_cam
            y_cam_imu = r_cam

            self.kf_roll.update(r_cam_imu)
            self.kf_pitch.update(p_cam_imu)
            self.kf_yaw.update(y_cam_imu)

        # Publish
        msg = Float32MultiArray()
        msg.data = [float(self.kf_roll.x), float(self.kf_pitch.x), float(self.kf_yaw.x)]

        self.pub_fusion.publish(msg)

        self.get_logger().info(
            f"FUSION → R:{self.kf_roll.x:.2f} "
            f"P:{self.kf_pitch.x:.2f} "
            f"Y:{self.kf_yaw.x:.2f}"
        )


# ============================================================
# MAIN
# ============================================================


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
