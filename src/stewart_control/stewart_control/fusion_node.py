#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from stewart_control.config_loader import get_config
from stewart_control.fusion_utils import Kalman1D, wrap_deg


class FusionNode(Node):
    def __init__(self):
        super().__init__("fusion_node")

        self.sub_pos = self.create_subscription(
            Float32MultiArray, "aruco_position", self.position_callback, 1
        )
        self.sub_imu = self.create_subscription(
            Float32MultiArray, "imu_error", self.imu_callback, 1
        )
        self.sub_cam = self.create_subscription(
            Float32MultiArray, "aruco_orientation", self.cam_callback, 1
        )

        self.pub_fusion = self.create_publisher(Float32MultiArray, "F_orientation", 1)
        self.pub_fused_position = self.create_publisher(
            Float32MultiArray, "F_position", 1
        )
        self.pub_fused_pose = self.create_publisher(Float32MultiArray, "F_pose", 1)

        cfg = get_config()
        fus = cfg["fusion"]
        auto_cfg = cfg.get("automatic_control", {})

        self.kf_roll = Kalman1D(q=fus["kalman_roll"]["q"], r=fus["kalman_roll"]["r"])
        self.kf_pitch = Kalman1D(q=fus["kalman_pitch"]["q"], r=fus["kalman_pitch"]["r"])
        self.kf_yaw = Kalman1D(q=fus["kalman_yaw"]["q"], r=fus["kalman_yaw"]["r"])
        self.camera_timeout_s = float(
            fus.get("camera_timeout_s", auto_cfg.get("fusion_timeout_s", 0.2))
        )
        self.orientation_hold_deadband_deg = float(
            fus.get("orientation_hold_deadband_deg", 0.4)
        )

        self.last_position = None
        self.last_imu = None
        self.last_cam = None
        self.last_position_time = None
        self.last_cam_time = None
        self.camera_pose_valid = False
        self.last_published_orientation = None

        self.get_logger().info(
            "Fusion node started. Expecting camera position [x, y, z], absolute IMU "
            "orientation [roll, pitch, yaw], and camera orientation [roll, pitch, yaw]."
        )

    def position_callback(self, msg):
        if len(msg.data) >= 3:
            self.last_position = list(msg.data[:3])
            self.last_position_time = time.monotonic()
            self.compute_fusion()

    def imu_callback(self, msg):
        self.last_imu = msg.data
        self.compute_fusion()

    def cam_callback(self, msg):
        self.last_cam = msg.data
        self.last_cam_time = time.monotonic()
        self.compute_fusion()

    def _camera_pose_is_fresh(self):
        if (
            self.last_position is None
            or self.last_cam is None
            or self.last_position_time is None
            or self.last_cam_time is None
        ):
            return False

        now = time.monotonic()
        return (
            now - self.last_position_time <= self.camera_timeout_s
            and now - self.last_cam_time <= self.camera_timeout_s
        )

    def _hold_small_orientation_changes(self, orientation):
        orientation = [wrap_deg(value) for value in orientation]
        if self.last_published_orientation is None:
            self.last_published_orientation = orientation
            return orientation

        held_orientation = []
        changed = False
        for previous, current in zip(self.last_published_orientation, orientation):
            if abs(wrap_deg(current - previous)) <= self.orientation_hold_deadband_deg:
                held_orientation.append(previous)
            else:
                held_orientation.append(current)
                changed = True

        if changed:
            self.last_published_orientation = held_orientation

        return held_orientation

    def compute_fusion(self):
        if self.last_imu is None:
            return

        r_imu, p_imu, y_imu = self.last_imu
        camera_pose_fresh = self._camera_pose_is_fresh()

        # Log sensor inputs for debugging
        if camera_pose_fresh:
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

        self.kf_roll.predict()
        self.kf_pitch.predict()
        self.kf_yaw.predict()
        self.kf_roll.update(wrap_deg(r_imu))
        self.kf_pitch.update(wrap_deg(p_imu))
        self.kf_yaw.update(wrap_deg(y_imu))

        if camera_pose_fresh:
            r_cam, p_cam, y_cam = self.last_cam
            self.kf_roll.update(wrap_deg(r_cam))
            self.kf_pitch.update(wrap_deg(p_cam))
            self.kf_yaw.update(wrap_deg(y_cam))
        elif self.camera_pose_valid:
            self.get_logger().warn(
                "Camera pose is stale or marker detection is lost. "
                "F_pose will not be published until camera detection recovers."
            )

        fused_orientation = self._hold_small_orientation_changes(
            [self.kf_roll.x, self.kf_pitch.x, self.kf_yaw.x]
        )

        msg = Float32MultiArray()
        msg.data = [float(value) for value in fused_orientation]
        self.pub_fusion.publish(msg)

        self.camera_pose_valid = camera_pose_fresh

        if camera_pose_fresh:
            pos_msg = Float32MultiArray()
            pos_msg.data = [float(value) for value in self.last_position]
            self.pub_fused_position.publish(pos_msg)

            pose_msg = Float32MultiArray()
            pose_msg.data = [
                float(self.last_position[0]),
                float(self.last_position[1]),
                float(self.last_position[2]),
                float(fused_orientation[0]),
                float(fused_orientation[1]),
                float(fused_orientation[2]),
            ]
            self.pub_fused_pose.publish(pose_msg)

        self.get_logger().debug(
            f"FUSION -> XYZ:{self.last_position if camera_pose_fresh else 'N/A'} "
            f"R:{fused_orientation[0]:.2f} "
            f"P:{fused_orientation[1]:.2f} "
            f"Y:{fused_orientation[2]:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
