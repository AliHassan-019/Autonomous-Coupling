#!/usr/bin/env python3
"""
Sensor Fusion Node - Combines IMU and Camera Orientation Data

WHY SENSORS DIFFER:
- IMU: Measures absolute orientation (gravity + magnetic north)
- Camera: Measures relative orientation (marker-to-marker rotation)
- Fusion: Kalman filter blends both for best accuracy

EXPECTED BEHAVIOR:
- Raw sensor values will differ significantly (50-180° on any axis)
- Fused output provides reliable orientation for control
- Use F_orientation topic for control, not individual sensors
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from stewart_control.config_loader import get_config
from stewart_control.fusion_utils import Kalman1D, wrap_deg


class FusionNode(Node):
    def __init__(self):
        super().__init__("fusion_node")

        self.sub_pos = self.create_subscription(
            Float32MultiArray, "aruco_position", self.position_callback, 10
        )
        self.sub_imu = self.create_subscription(
            Float32MultiArray, "imu_error", self.imu_callback, 10
        )
        self.sub_cam = self.create_subscription(
            Float32MultiArray, "aruco_orientation", self.cam_callback, 10
        )

        self.pub_fusion = self.create_publisher(Float32MultiArray, "F_orientation", 10)
        self.pub_fused_position = self.create_publisher(
            Float32MultiArray, "F_position", 10
        )
        self.pub_fused_pose = self.create_publisher(Float32MultiArray, "F_pose", 10)

        cfg = get_config()
        fus = cfg["fusion"]

        self.kf_roll = Kalman1D(q=fus["kalman_roll"]["q"], r=fus["kalman_roll"]["r"])
        self.kf_pitch = Kalman1D(q=fus["kalman_pitch"]["q"], r=fus["kalman_pitch"]["r"])
        self.kf_yaw = Kalman1D(q=fus["kalman_yaw"]["q"], r=fus["kalman_yaw"]["r"])

        self.last_position = None
        self.last_imu = None
        self.last_cam = None

        self.get_logger().info(
            "Fusion node started. Expecting camera position [x, y, z], absolute IMU "
            "orientation [roll, pitch, yaw], and camera orientation [roll, pitch, yaw]."
        )

    def position_callback(self, msg):
        if len(msg.data) >= 3:
            self.last_position = list(msg.data[:3])
            self.compute_fusion()

    def imu_callback(self, msg):
        self.last_imu = msg.data
        self.compute_fusion()

    def cam_callback(self, msg):
        self.last_cam = msg.data
        self.compute_fusion()

    def compute_fusion(self):
        if self.last_imu is None:
            return

        r_imu, p_imu, y_imu = self.last_imu

        # Log sensor inputs for debugging
        if self.last_cam is not None:
            r_cam, p_cam, y_cam = self.last_cam
            self.get_logger().debug(
                f"FUSION INPUTS - IMU: R={r_imu:.2f} P={p_imu:.2f} Y={y_imu:.2f} | "
                f"CAMERA: R={r_cam:.2f} P={p_cam:.2f} Y={y_cam:.2f}"
            )
        else:
            self.get_logger().debug(
                f"FUSION INPUTS - IMU: R={r_imu:.2f} P={p_imu:.2f} Y={y_imu:.2f} | "
                f"CAMERA: N/A"
            )

        self.kf_roll.predict(wrap_deg(r_imu))
        self.kf_pitch.predict(wrap_deg(p_imu))
        self.kf_yaw.predict(wrap_deg(y_imu))

        if self.last_cam is not None:
            r_cam, p_cam, y_cam = self.last_cam
            self.kf_roll.update(wrap_deg(r_cam))
            self.kf_pitch.update(wrap_deg(p_cam))
            self.kf_yaw.update(wrap_deg(y_cam))

        msg = Float32MultiArray()
        msg.data = [float(self.kf_roll.x), float(self.kf_pitch.x), float(self.kf_yaw.x)]
        self.pub_fusion.publish(msg)

        if self.last_position is not None:
            pos_msg = Float32MultiArray()
            pos_msg.data = [float(value) for value in self.last_position]
            self.pub_fused_position.publish(pos_msg)

            pose_msg = Float32MultiArray()
            pose_msg.data = [
                float(self.last_position[0]),
                float(self.last_position[1]),
                float(self.last_position[2]),
                float(self.kf_roll.x),
                float(self.kf_pitch.x),
                float(self.kf_yaw.x),
            ]
            self.pub_fused_pose.publish(pose_msg)

        self.get_logger().info(
            f"FUSION -> XYZ:{self.last_position if self.last_position is not None else 'N/A'} "
            f"R:{self.kf_roll.x:.2f} "
            f"P:{self.kf_pitch.x:.2f} "
            f"Y:{self.kf_yaw.x:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
