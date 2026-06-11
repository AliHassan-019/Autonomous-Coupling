#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from stewart_control.config_loader import get_config
from stewart_control.fusion_utils import OrientationKalmanFilter, wrap_deg


class FusionNode(Node):
    def __init__(self):
        super().__init__("fusion_node")

        self.sub_pos = self.create_subscription(
            Float32MultiArray, "aruco_position", self.position_callback, 1
        )
        self.sub_imu = self.create_subscription(
            Float32MultiArray, "imu_error", self.imu_callback, 1
        )
        self.sub_gyro = self.create_subscription(
            Float32MultiArray, "imu_gyro", self.gyro_callback, 1
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

        self.axis_filter_configs = self._load_axis_filter_configs(fus)
        self.orientation_filter = OrientationKalmanFilter(self.axis_filter_configs)
        self.imu_measurement_variances = [
            self.axis_filter_configs[axis]["imu_r"]
            for axis in OrientationKalmanFilter.AXES
        ]
        self.camera_measurement_variances = [
            self.axis_filter_configs[axis]["camera_r"]
            for axis in OrientationKalmanFilter.AXES
        ]
        self.gyro_measurement_variances = [
            self.axis_filter_configs[axis]["gyro_r"]
            for axis in OrientationKalmanFilter.AXES
        ]
        self.use_gyro_prediction = bool(fus.get("use_gyro_prediction", True))
        self.gyro_timeout_s = float(fus.get("gyro_timeout_s", 0.12))
        self.camera_timeout_s = float(
            fus.get("camera_timeout_s", auto_cfg.get("fusion_timeout_s", 0.2))
        )
        self.orientation_hold_deadband_deg = float(
            fus.get("orientation_hold_deadband_deg", 0.4)
        )
        self.max_filter_dt_s = float(fus.get("max_filter_dt_s", 0.1))

        self.last_position = None
        self.last_imu = None
        self.last_gyro = None
        self.last_cam = None
        self.last_position_time = None
        self.last_gyro_time = None
        self.last_cam_time = None
        self.last_filter_time = None
        self.camera_pose_valid = False
        self.last_published_orientation = None

        self.get_logger().info(
            "Fusion node started. Expecting camera position [x, y, z], absolute IMU "
            "orientation [roll, pitch, yaw], and camera orientation [roll, pitch, yaw]."
        )
        self.get_logger().info(
            "Using angle-rate Kalman fusion with IMU/camera measurement weighting "
            "and camera outlier rejection."
        )
        self.get_logger().info(
            "Gyro prediction is "
            f"{'enabled' if self.use_gyro_prediction else 'disabled'} "
            f"with timeout {self.gyro_timeout_s:.3f}s."
        )

    @staticmethod
    def _axis_config(fusion_cfg, axis_name, default_q, default_r):
        legacy = fusion_cfg.get(f"kalman_{axis_name}", {})
        return {
            "process_angle_q": float(
                legacy.get("process_angle_q", legacy.get("q", default_q))
            ),
            "process_rate_q": float(legacy.get("process_rate_q", default_q * 120.0)),
            "imu_r": float(legacy.get("imu_r", legacy.get("r", default_r))),
            "camera_r": float(legacy.get("camera_r", legacy.get("r", default_r))),
            "gyro_r": float(legacy.get("gyro_r", 4.0)),
            "outlier_threshold_deg": float(legacy.get("outlier_threshold_deg", 8.0)),
            "outlier_recovery_count": int(legacy.get("outlier_recovery_count", 3)),
        }

    def _load_axis_filter_configs(self, fusion_cfg):
        return {
            "roll": self._axis_config(fusion_cfg, "roll", 0.01, 1.5),
            "pitch": self._axis_config(fusion_cfg, "pitch", 0.01, 1.5),
            "yaw": self._axis_config(fusion_cfg, "yaw", 0.01, 5.0),
        }

    def position_callback(self, msg):
        if len(msg.data) >= 3:
            self.last_position = list(msg.data[:3])
            self.last_position_time = time.monotonic()
            self.compute_fusion()

    def imu_callback(self, msg):
        self.last_imu = msg.data
        self.compute_fusion()

    def gyro_callback(self, msg):
        if len(msg.data) >= 3:
            self.last_gyro = list(msg.data[:3])
            self.last_gyro_time = time.monotonic()

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

    def _gyro_is_fresh(self, now):
        return (
            self.use_gyro_prediction
            and self.last_gyro is not None
            and self.last_gyro_time is not None
            and now - self.last_gyro_time <= self.gyro_timeout_s
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

        now = time.monotonic()
        if self.last_filter_time is None:
            dt = 1e-3
        else:
            dt = min(now - self.last_filter_time, self.max_filter_dt_s)
        self.last_filter_time = now

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

        self.orientation_filter.predict(dt)
        if self._gyro_is_fresh(now):
            self.orientation_filter.update_rates(
                self.last_gyro,
                self.gyro_measurement_variances,
            )
        self.orientation_filter.update(
            [wrap_deg(r_imu), wrap_deg(p_imu), wrap_deg(y_imu)],
            self.imu_measurement_variances,
            allow_outlier_recovery=True,
        )

        if camera_pose_fresh:
            r_cam, p_cam, y_cam = self.last_cam
            accepted = self.orientation_filter.update(
                [wrap_deg(r_cam), wrap_deg(p_cam), wrap_deg(y_cam)],
                self.camera_measurement_variances,
                allow_outlier_recovery=False,
            )
            if not all(accepted):
                rejected_axes = [
                    axis
                    for axis, is_accepted in zip(
                        OrientationKalmanFilter.AXES, accepted
                    )
                    if not is_accepted
                ]
                self.get_logger().debug(
                    "Rejected camera orientation outlier axis/axes: "
                    f"{', '.join(rejected_axes)}"
                )
        elif self.camera_pose_valid:
            self.get_logger().warn(
                "Camera pose is stale or marker detection is lost. "
                "F_pose will not be published until camera detection recovers."
            )

        fused_orientation = self._hold_small_orientation_changes(
            self.orientation_filter.orientation
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
            f"Y:{fused_orientation[2]:.2f} "
            f"rates:{[round(value, 2) for value in self.orientation_filter.rates]}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
