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
import threading
import time
from ament_index_python.packages import get_package_share_directory
from stewart_control.config_loader import get_config

BACKEND_MAP = {
    "any": cv2.CAP_ANY,
    "dshow": getattr(cv2, "CAP_DSHOW", cv2.CAP_ANY),
    "msmf": getattr(cv2, "CAP_MSMF", cv2.CAP_ANY),
    "v4l2": getattr(cv2, "CAP_V4L2", cv2.CAP_ANY),
}


def create_detector_parameters():
    if hasattr(aruco, "DetectorParameters"):
        parameters = aruco.DetectorParameters()
    else:
        parameters = aruco.DetectorParameters_create()

    if hasattr(aruco, "CORNER_REFINE_SUBPIX"):
        parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    if hasattr(parameters, "cornerRefinementWinSize"):
        parameters.cornerRefinementWinSize = 5
    if hasattr(parameters, "cornerRefinementMaxIterations"):
        parameters.cornerRefinementMaxIterations = 50
    if hasattr(parameters, "cornerRefinementMinAccuracy"):
        parameters.cornerRefinementMinAccuracy = 0.01
    if hasattr(parameters, "adaptiveThreshWinSizeMin"):
        parameters.adaptiveThreshWinSizeMin = 5
    if hasattr(parameters, "adaptiveThreshWinSizeMax"):
        parameters.adaptiveThreshWinSizeMax = 31
    if hasattr(parameters, "adaptiveThreshWinSizeStep"):
        parameters.adaptiveThreshWinSizeStep = 4
    if hasattr(parameters, "minMarkerPerimeterRate"):
        parameters.minMarkerPerimeterRate = 0.02
        
    if hasattr(parameters, "maxMarkerPerimeterRate"):
        parameters.maxMarkerPerimeterRate = 4.0
    if hasattr(parameters, "minCornerDistanceRate"):
        parameters.minCornerDistanceRate = 0.03
    if hasattr(parameters, "minDistanceToBorder"):
        parameters.minDistanceToBorder = 3
    return parameters


def apply_camera_settings(cap, camera_cfg):
    if "width" in camera_cfg:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(camera_cfg["width"]))
    if "height" in camera_cfg:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(camera_cfg["height"]))
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    auto_exposure = camera_cfg.get("auto_exposure")
    if auto_exposure in ("on", "off"):
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75 if auto_exposure == "on" else 0.25)

    auto_wb = camera_cfg.get("auto_wb")
    if auto_wb in ("on", "off"):
        cap.set(cv2.CAP_PROP_AUTO_WB, 1.0 if auto_wb == "on" else 0.0)

    autofocus = camera_cfg.get("autofocus")
    if autofocus in ("on", "off") and hasattr(cv2, "CAP_PROP_AUTOFOCUS"):
        cap.set(cv2.CAP_PROP_AUTOFOCUS, 1.0 if autofocus == "on" else 0.0)

    focus = camera_cfg.get("focus")
    if focus is not None and hasattr(cv2, "CAP_PROP_FOCUS"):
        cap.set(cv2.CAP_PROP_FOCUS, float(focus))

    for key, prop_id in (
        ("exposure", cv2.CAP_PROP_EXPOSURE),
        ("gain", cv2.CAP_PROP_GAIN),
        ("brightness", cv2.CAP_PROP_BRIGHTNESS),
        ("contrast", cv2.CAP_PROP_CONTRAST),
        ("saturation", cv2.CAP_PROP_SATURATION),
        ("sharpness", cv2.CAP_PROP_SHARPNESS),
        ("wb_temperature", cv2.CAP_PROP_WB_TEMPERATURE),
    ):
        value = camera_cfg.get(key)
        if value is not None:
            cap.set(prop_id, float(value))


def normalize_angle_deg(angle):
    return ((float(angle) + 180.0) % 360.0) - 180.0


def unwrap_angle_deg(current, previous):
    current = normalize_angle_deg(current)
    previous = normalize_angle_deg(previous)
    delta = normalize_angle_deg(current - previous)
    return previous + delta


def wrap_rpy_deg(values):
    return np.array([normalize_angle_deg(value) for value in values], dtype=float)


def draw_marker_boxes(frame, corners):
    for marker_corners in corners:
        points = np.asarray(marker_corners, dtype=np.int32).reshape(-1, 2)
        cv2.polylines(frame, [points], isClosed=True, color=(0, 255, 0), thickness=2)


class ArucoRelativePose(Node):
    CAMERA_READ_WARN_S = 0.5
    CAMERA_REOPEN_DELAY_S = 0.25

    def __init__(self):
        super().__init__("aruco_relative_pose")

        # ------ Publishers ------
        # Tes anciens noms:
        self.pub_pos = self.create_publisher(Float32MultiArray, "aruco_position", 1)
        self.pub_ori = self.create_publisher(Float32MultiArray, "aruco_orientation", 1)
        self.pub_img = self.create_publisher(Image, "camera/image_raw", 1)

        self.bridge = CvBridge()

        # ------ caméra ------
        cfg = get_config()
        aruco_cfg = cfg["aruco"]
        camera_cfg = {
            "width": aruco_cfg.get("camera_width", 640),
            "height": aruco_cfg.get("camera_height", 480),
            "auto_exposure": aruco_cfg.get("camera_auto_exposure"),
            "exposure": aruco_cfg.get("camera_exposure"),
            "gain": aruco_cfg.get("camera_gain"),
            "brightness": aruco_cfg.get("camera_brightness"),
            "contrast": aruco_cfg.get("camera_contrast"),
            "saturation": aruco_cfg.get("camera_saturation"),
            "sharpness": aruco_cfg.get("camera_sharpness"),
            "auto_wb": aruco_cfg.get("camera_auto_wb"),
            "wb_temperature": aruco_cfg.get("camera_wb_temperature"),
            "autofocus": aruco_cfg.get("camera_autofocus"),
            "focus": aruco_cfg.get("camera_focus"),
        }

        camera_index = int(aruco_cfg.get("camera_index", 0))
        camera_backend = str(aruco_cfg.get("camera_backend", "any")).lower()
        self.camera_index = camera_index
        self.camera_backend = BACKEND_MAP.get(camera_backend, cv2.CAP_ANY)
        self.camera_cfg = camera_cfg
        self.camera_reopen_failures = int(aruco_cfg.get("camera_reopen_failures", 3))
        self.camera_watchdog_timeout_s = float(
            aruco_cfg.get("camera_watchdog_timeout_s", 6.0)
        )
        self.camera_read_failures = 0
        self.last_camera_frame_time = time.monotonic()
        self._watchdog_stop = threading.Event()

        self.cap = self._open_camera()
        if not self.cap.isOpened():
            self.get_logger().error("Erreur : impossible d'ouvrir la caméra.")
            return

        # ------ ArUco ------
        aruco_dict_id = getattr(aruco, aruco_cfg["dictionary"])
        self.aruco_dict = aruco.getPredefinedDictionary(aruco_dict_id)
        self.parameters = create_detector_parameters()

        # ------ Charger calibration intrinsèque + extrinsèque ------
        pkg = get_package_share_directory("stewart_control")

        intr_path = os.path.join(pkg, "calib_int.npz")
        extr_path = os.path.join(pkg, "calib_ext3.npz")

        intr = np.load(intr_path)
        self.mtx = intr["mtx"]
        self.dist = intr["dist"]
        self.marker_size = aruco_cfg["marker_size"]
        self.single_marker_only = bool(aruco_cfg.get("single_marker_only", False))
        self.publish_in_base_frame = bool(aruco_cfg.get("publish_in_base_frame", False))
        self.reference_position = np.array(
            aruco_cfg.get("reference_position_m", [0.0, 0.0, 0.0]),
            dtype=float,
        )
        self.reference_orientation = wrap_rpy_deg(
            aruco_cfg.get("reference_orientation_deg", [0.0, 0.0, 0.0])
        )
        # Camera axis remapping (similar to IMU)
        self.orientation_axis_order = tuple(int(value) for value in aruco_cfg.get("orientation_axis_order", [0, 1, 2]))
        self.orientation_axis_sign = tuple(float(value) for value in aruco_cfg.get("orientation_axis_sign", [1, 1, 1]))
        self.pose_alpha = float(aruco_cfg.get("pose_alpha", 0.22))
        self.position_deadband_m = float(aruco_cfg.get("position_deadband_m", 0.0015))
        self.orientation_deadband_deg = float(
            aruco_cfg.get("orientation_deadband_deg", 0.35)
        )
        self.hold_last_frames = int(aruco_cfg.get("hold_last_frames", 4))
        self.last_position = None
        self.last_orientation = None
        self.last_marker_corners = None
        self.frames_since_detection = 0

        extr = np.load(extr_path)
        self.rvec_ext = extr["rvec"].reshape(3, 1)
        self.tvec_ext = extr["tvec"].reshape(3, 1)
        self.R_ext, _ = cv2.Rodrigues(self.rvec_ext)

        # IDs
        self.fixed_id = aruco_cfg["fixed_marker_id"]
        self.mobile_id = aruco_cfg.get("moving_marker_id", aruco_cfg["mobile_marker_id"])

        # Timer ROS2
        self.timer = self.create_timer(aruco_cfg["loop_rate"], self.loop)
        self._watchdog_thread = threading.Thread(
            target=self._camera_watchdog_loop,
            name="aruco-camera-watchdog",
            daemon=True,
        )
        self._watchdog_thread.start()
        self.get_logger().info(
            "Using camera reference pose "
            f"XYZ={self.reference_position.tolist()} m "
        
        
            f"RPY={self.reference_orientation.tolist()} deg."
        )

    def _open_camera(self):
        cap = cv2.VideoCapture(self.camera_index, self.camera_backend)
        if hasattr(cv2, "CAP_PROP_READ_TIMEOUT_MSEC"):
            cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 1000)
        if hasattr(cv2, "CAP_PROP_OPEN_TIMEOUT_MSEC"):
            cap.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 1000)
        apply_camera_settings(cap, self.camera_cfg)
        return cap

    def _reopen_camera(self):
        self.get_logger().warn("Camera read failed repeatedly; reopening camera.")
        try:
            self.cap.release()
        except Exception as exc:
            self.get_logger().warn(f"Camera release during reopen failed: {exc}")

        time.sleep(self.CAMERA_REOPEN_DELAY_S)
        self.cap = self._open_camera()
        self.camera_read_failures = 0
        if not self.cap.isOpened():
            self.get_logger().error("Camera reopen failed.")
            return False

        self.last_camera_frame_time = time.monotonic()
        self.get_logger().info("Camera reopened successfully.")
        return True

    def _read_camera_frame(self):
        started = time.monotonic()
        ret, frame = self.cap.read()
        elapsed = time.monotonic() - started

        if elapsed > self.CAMERA_READ_WARN_S:
            self.get_logger().warn(
                f"Slow camera read: {elapsed:.2f}s. USB/V4L2 may be stalling."
            )

        if ret:
            self.camera_read_failures = 0
            self.last_camera_frame_time = time.monotonic()
            return True, frame

        self.camera_read_failures += 1
        if self.camera_read_failures >= self.camera_reopen_failures:
            self._reopen_camera()

        return False, None

    def _camera_watchdog_loop(self):
        while not self._watchdog_stop.wait(1.0):
            stale_s = time.monotonic() - self.last_camera_frame_time
            if stale_s <= self.camera_watchdog_timeout_s:
                continue

            self.get_logger().fatal(
                "Camera watchdog timeout: no frame published for "
                f"{stale_s:.1f}s. Exiting aruco_node so launch can respawn it."
            )
            os._exit(3)

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

    @staticmethod
    def remap_rpy_deg(values, axis_order=(0, 1, 2), axis_sign=(1.0, 1.0, 1.0)):
        """Remap camera orientation axes (similar to IMU remapping)."""
        wrapped = wrap_rpy_deg(values)
        signs = np.array(axis_sign, dtype=float)
        return wrap_rpy_deg(wrapped[list(axis_order)] * signs)

    @staticmethod
    def _smooth_vector(previous, current, alpha):
        alpha = min(max(float(alpha), 0.0), 1.0)
        previous = np.array(previous, dtype=float)
        current = np.array(current, dtype=float)
        return (1.0 - alpha) * previous + alpha * current

    @staticmethod
    def _apply_deadband(previous, current, threshold):
        threshold = max(float(threshold), 0.0)
        previous = np.array(previous, dtype=float)
        current = np.array(current, dtype=float)
        held = previous.copy()
        update_mask = np.abs(current - previous) > threshold
        held[update_mask] = current[update_mask]
        return held

    def _stabilize_pose(self, position, orientation_deg, marker_corners):
        position = np.array(position, dtype=float)
        orientation = np.array(orientation_deg, dtype=float)

        if self.last_position is None:
            stabilized_position = position
            stabilized_orientation = np.array(
                [normalize_angle_deg(value) for value in orientation],
                dtype=float,
            )
        else:
            stabilized_position = self._smooth_vector(
                self.last_position,
                position,
                self.pose_alpha,
            )
            unwrapped_orientation = np.array(
                [
                    unwrap_angle_deg(current, previous)
                    for current, previous in zip(orientation, self.last_orientation)
                ],
                dtype=float,
            )
            stabilized_orientation = self._smooth_vector(
                self.last_orientation,
                unwrapped_orientation,
                self.pose_alpha,
            )
            stabilized_position = self._apply_deadband(
                self.last_position,
                stabilized_position,
                self.position_deadband_m,
            )
            stabilized_orientation = self._apply_deadband(
                self.last_orientation,
                stabilized_orientation,
                self.orientation_deadband_deg,
            )

        self.last_position = stabilized_position
        self.last_orientation = stabilized_orientation
        self.last_marker_corners = marker_corners
        self.frames_since_detection = 0
        return stabilized_position, stabilized_orientation

    def _apply_reference_pose(self, position, orientation_deg):
        # Log raw orientation values before any processing (for debugging)
        raw_orientation = np.array(orientation_deg, dtype=float)
        self.get_logger().debug(
            f"CAMERA RAW: Roll={raw_orientation[0]:.2f} "
            f"Pitch={raw_orientation[1]:.2f} Yaw={raw_orientation[2]:.2f}"
        )

        # Apply axis remapping (similar to IMU)
        remapped_orientation = self.remap_rpy_deg(
            raw_orientation,
            axis_order=self.orientation_axis_order,
            axis_sign=self.orientation_axis_sign
        )

        # Apply reference subtraction
        corrected_position = np.array(position, dtype=float) - self.reference_position
        corrected_orientation = wrap_rpy_deg(remapped_orientation - self.reference_orientation)

        self.get_logger().debug(
            f"CAMERA PROCESSED: Roll={corrected_orientation[0]:.2f} "
            f"Pitch={corrected_orientation[1]:.2f} Yaw={corrected_orientation[2]:.2f}"
        )

        return corrected_position, corrected_orientation

    def _publish_stabilized_pose(self, position, orientation_deg):
        msg_pos = Float32MultiArray()
        msg_pos.data = [float(value) for value in np.asarray(position, dtype=float)]
        self.pub_pos.publish(msg_pos)

        msg_ori = Float32MultiArray()
        msg_ori.data = [
            float(normalize_angle_deg(value))
            for value in np.asarray(orientation_deg, dtype=float)
        ]
        self.pub_ori.publish(msg_ori)

    def _publish_held_pose(self):
        if self.last_position is None or self.last_orientation is None:
            return False
        if self.frames_since_detection >= self.hold_last_frames:
            return False
        self.frames_since_detection += 1
        self._publish_stabilized_pose(self.last_position, self.last_orientation)
        return True

    def loop(self):
        ret, frame = self._read_camera_frame()
        if not ret:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.parameters
        )

        camera_ready = False

        if ids is not None:
            ids_flat = ids.flatten()
            draw_marker_boxes(frame, corners)

            if self.single_marker_only and self.mobile_id in ids_flat:
                idx = ids_flat.tolist().index(self.mobile_id)
                rvec_m, tvec_m, _ = aruco.estimatePoseSingleMarkers(
                    [corners[idx]], self.marker_size, self.mtx, self.dist
                )
                rvec_m = rvec_m[0].reshape(3, 1)
                tvec_m = tvec_m[0].reshape(3, 1)

                if self.publish_in_base_frame:
                    R_m, _ = cv2.Rodrigues(rvec_m)
                    t_pose = self.R_ext @ tvec_m + self.tvec_ext
                    R_pose = self.R_ext @ R_m
                    rvec_pose, _ = cv2.Rodrigues(R_pose)
                    
                else:
                    t_pose = tvec_m
                    rvec_pose = rvec_m

                t_marker, marker_orientation = self._apply_reference_pose(
                    t_pose.flatten(),
                    self.rvec_to_euler(rvec_pose),
                )
                stabilized_position, stabilized_orientation = self._stabilize_pose(
                    t_marker,
                    marker_orientation,
                    corners[idx],
                )
                self._publish_stabilized_pose(
                    stabilized_position,
                    stabilized_orientation,
                )

                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.pub_img.publish(img_msg)
                return

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

                t_mobile_in_fixed, mobile_orientation = self._apply_reference_pose(
                    t_m_world.flatten(),
                    self.rvec_to_euler(cv2.Rodrigues(R_m_world)[0]),
                )
                stabilized_position, stabilized_orientation = self._stabilize_pose(
                    t_mobile_in_fixed,
                    mobile_orientation,
                    corners[idx],
                )
                self._publish_stabilized_pose(
                    stabilized_position,
                    stabilized_orientation,
                )

        if ids is None:
            self._publish_held_pose()

        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.pub_img.publish(img_msg)

    def destroy_node(self):
        if hasattr(self, "_watchdog_stop"):
            self._watchdog_stop.set()
        if hasattr(self, "_watchdog_thread") and self._watchdog_thread.is_alive():
            self._watchdog_thread.join(timeout=1.0)
        if hasattr(self, "cap"):
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
