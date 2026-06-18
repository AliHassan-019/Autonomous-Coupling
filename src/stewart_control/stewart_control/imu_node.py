#!/home/rem/rtimulib-env/bin/python3

import math
import os
from pathlib import Path

import numpy as np
import rclpy
import smbus
from ament_index_python.packages import get_package_share_directory
from imusensor.MPU9250 import MPU9250
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from stewart_control.config_loader import get_config


def compute_rpy(imu):
    ax, ay, az = imu.AccelVals
    mx, my, mz = imu.MagVals

    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))

    xh = mx * math.cos(pitch) + mz * math.sin(pitch)
    yh = (
        mx * math.sin(roll) * math.sin(pitch)
        + my * math.cos(roll)
        - mz * math.sin(roll) * math.cos(pitch)
    )
    yaw = math.atan2(-yh, xh)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def wrap_deg(angle: float) -> float:
    return (float(angle) + 180.0) % 360.0 - 180.0


def wrap_rpy_deg(values) -> np.ndarray:
    return np.array([wrap_deg(value) for value in values], dtype=float)


def normalize_axis_order(values) -> tuple[int, int, int]:
    order = tuple(int(value) for value in values)
    if len(order) != 3 or set(order) != {0, 1, 2}:
        raise ValueError(
            "IMU orientation_axis_order must contain each axis index 0, 1, 2 exactly once."
        )
    return order


def normalize_axis_signs(values) -> tuple[float, float, float]:
    signs = tuple(float(value) for value in values)
    if len(signs) != 3 or any(value not in (-1.0, 1.0) for value in signs):
        raise ValueError(
            "IMU orientation_axis_sign must contain exactly three values, each either -1 or 1."
        )
    return signs


def remap_rpy_deg(
    values,
    axis_order=(0, 1, 2),
    axis_sign=(1.0, 1.0, 1.0),
) -> np.ndarray:
    wrapped = wrap_rpy_deg(values)
    order = normalize_axis_order(axis_order)
    signs = np.array(normalize_axis_signs(axis_sign), dtype=float)
    return wrap_rpy_deg(wrapped[list(order)] * signs)


def remap_vector(values, axis_order=(0, 1, 2), axis_sign=(1.0, 1.0, 1.0)) -> np.ndarray:
    vector = np.array(values, dtype=float)
    order = normalize_axis_order(axis_order)
    signs = np.array(normalize_axis_signs(axis_sign), dtype=float)
    return vector[list(order)] * signs


def resolve_calibration_path(configured_path: str, filename: str) -> str:
    candidates = []

    if configured_path:
        configured = Path(os.path.expanduser(configured_path))
        candidates.append(configured)

        if not configured.is_absolute():
            package_root = Path(__file__).resolve().parents[1]
            candidates.append(package_root / configured)
            candidates.append(package_root / "share" / "stewart_control" / configured)

    try:
        share_dir = Path(get_package_share_directory("stewart_control"))
        candidates.append(share_dir / filename)
        candidates.append(share_dir / "share" / "stewart_control" / filename)
    except Exception:
        pass

    package_root = Path(__file__).resolve().parents[1]
    candidates.append(package_root / "share" / "stewart_control" / filename)
    candidates.append(package_root / filename)
    candidates.append(Path.cwd() / "src" / "stewart_control" / "share" / "stewart_control" / filename)

    seen = set()
    for candidate in candidates:
        candidate = candidate.resolve()
        if str(candidate) in seen:
            continue
        seen.add(str(candidate))
        if candidate.is_file():
            return str(candidate)

    searched = "\n".join(f" - {path}" for path in seen)
    raise FileNotFoundError(
        
        f"Unable to find IMU calibration file '{filename}'. Searched:\n{searched}"
    )


class IMUPublisher(Node):
    def __init__(self):
        super().__init__("imu_error_node")

        cfg = get_config()
        imu_cfg = cfg["imu"]

        self.pub_imu = self.create_publisher(Float32MultiArray, "imu_error", 1)
        self.pub_gyro = self.create_publisher(Float32MultiArray, "imu_gyro", 1)

        bus = smbus.SMBus(imu_cfg["i2c_bus"])
        self.imu = MPU9250.MPU9250(bus, imu_cfg["imu_address"])
        self.imu.begin()

        calibration_path = resolve_calibration_path(
            imu_cfg.get("calibration_path"),
            "calib2.json",
        )

        self.imu.loadCalibDataFromFile(calibration_path)
        self.reference_orientation = wrap_rpy_deg(
            imu_cfg.get("reference_orientation_deg", [0.0, 0.0, 0.0])
            
        )
        self.orientation_axis_order = normalize_axis_order(
            imu_cfg.get("orientation_axis_order", [0, 1, 2])
        )
        self.orientation_axis_sign = normalize_axis_signs(
            imu_cfg.get("orientation_axis_sign", [1, 1, 1])
        )
        self.publish_gyro = bool(imu_cfg.get("publish_gyro", True))

        self.gyro_units = str(imu_cfg.get("gyro_units", "deg_s")).lower()
        if self.gyro_units not in ("deg_s", "rad_s"):
            raise ValueError("imu.gyro_units must be either 'deg_s' or 'rad_s'.")

        self.get_logger().info("Single IMU initialized and calibrated.")
        self.get_logger().info(
            f"Using IMU address 0x{int(imu_cfg['imu_address']):02X} "
            f"with calibration file {calibration_path}."
        )
        self.get_logger().info(
            "Using IMU orientation reference "
            f"[R={self.reference_orientation[0]:.2f}, "
            f"P={self.reference_orientation[1]:.2f}, "
            f"Y={self.reference_orientation[2]:.2f}] deg."
        )
        self.get_logger().info(
            "Using IMU orientation remap "
            f"order={list(self.orientation_axis_order)} "
            f"sign={list(self.orientation_axis_sign)}."
        )
        self.get_logger().info(
            "IMU gyro publishing is "
            f"{'enabled' if self.publish_gyro else 'disabled'} "
            f"with source units '{self.gyro_units}'."
        )

        self.timer = self.create_timer(imu_cfg["publish_rate"], self.publish_imu_error)

    def publish_imu_error(self):
        self.imu.readSensor()
        raw_rpy = np.array(compute_rpy(self.imu), dtype=float)
        raw_gyro = np.array(self.imu.GyroVals, dtype=float)

        # Log raw IMU values before any processing (for debugging)
        self.get_logger().debug(
            f"IMU RAW: Roll={raw_rpy[0]:.2f} "
            f"Pitch={raw_rpy[1]:.2f} Yaw={raw_rpy[2]:.2f}"
        )

        remapped_rpy = remap_rpy_deg(
            raw_rpy,
            axis_order=self.orientation_axis_order,
            axis_sign=self.orientation_axis_sign,
        )
        corrected_rpy = wrap_rpy_deg(remapped_rpy - self.reference_orientation)
        roll_abs, pitch_abs, yaw_abs = tuple(
            corrected_rpy
        )

        if self.publish_gyro:
            gyro_dps = remap_vector(
                raw_gyro,
                axis_order=self.orientation_axis_order,
                axis_sign=self.orientation_axis_sign,
            )
            if self.gyro_units == "rad_s":
                gyro_dps = np.rad2deg(gyro_dps)

            gyro_msg = Float32MultiArray()
            gyro_msg.data = [round(float(value), 3) for value in gyro_dps]
            self.pub_gyro.publish(gyro_msg)

        msg = Float32MultiArray()
        msg.data = [round(roll_abs, 2), round(pitch_abs, 2), round(yaw_abs, 2)]
        self.pub_imu.publish(msg)

        self.get_logger().debug(
            f"IMU PROCESSED: Roll: {roll_abs:.2f}, "
            f"Pitch: {pitch_abs:.2f}, Yaw: {yaw_abs:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = IMUPublisher()
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
