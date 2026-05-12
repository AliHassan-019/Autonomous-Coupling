#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
import re
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))


@dataclass
class DetectionResult:
    position_m: np.ndarray
    orientation_deg: tuple[float, float, float]
    fixed_rvec: np.ndarray
    fixed_tvec: np.ndarray
    mobile_rvec: np.ndarray
    mobile_tvec: np.ndarray


@dataclass
class SmoothedPose:
    position_m: np.ndarray
    orientation_deg: tuple[float, float, float]
    stale_frames: int


DEFAULT_ARUCO_CONFIG = {
    "dictionary": "DICT_4X4_1000",
    "marker_size": 0.022,
    "fixed_marker_id": 24,
    "mobile_marker_id": 28,
}

DEFAULT_BACKEND = {
    "any": cv2.CAP_ANY,
    "dshow": getattr(cv2, "CAP_DSHOW", cv2.CAP_ANY),
    "msmf": getattr(cv2, "CAP_MSMF", cv2.CAP_ANY),
    "v4l2": getattr(cv2, "CAP_V4L2", cv2.CAP_ANY),
}

DEFAULT_FOURCC = {
    "YUY2": cv2.VideoWriter_fourcc(*"YUY2"),
    "MJPG": cv2.VideoWriter_fourcc(*"MJPG"),
}


def create_detector_parameters():
    if hasattr(cv2.aruco, "DetectorParameters"):
        parameters = cv2.aruco.DetectorParameters()
    else:
        parameters = cv2.aruco.DetectorParameters_create()

    # Favor stable detection on moving targets by refining corners and keeping
    # thresholding reasonably tolerant to changing lighting.
    if hasattr(cv2.aruco, "CORNER_REFINE_SUBPIX"):
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
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


def detect_markers(gray: np.ndarray, dictionary, parameters):
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        return detector.detectMarkers(gray)
    return cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)


def draw_axes(frame: np.ndarray, camera_matrix: np.ndarray, dist_coeffs: np.ndarray, rvec, tvec):
    if hasattr(cv2.aruco, "drawAxis"):
        cv2.aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.05)
        return
    cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.05)


def rvec_to_euler_deg(rvec: np.ndarray) -> tuple[float, float, float]:
    rotation_matrix, _ = cv2.Rodrigues(rvec)
    sy = math.sqrt(rotation_matrix[0, 0] ** 2 + rotation_matrix[1, 0] ** 2)
    singular = sy < 1e-6

    if not singular:
        roll = math.atan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
        pitch = math.atan2(-rotation_matrix[2, 0], sy)
        yaw = math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    else:
        roll = math.atan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
        pitch = math.atan2(-rotation_matrix[2, 0], sy)
        yaw = 0.0

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def estimate_single_marker_pose(corner, marker_size: float, camera_matrix: np.ndarray, dist_coeffs: np.ndarray):
    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
        [corner], marker_size, camera_matrix, dist_coeffs
    )
    rvec = rvecs[0].reshape(3, 1)
    tvec = tvecs[0].reshape(3, 1)
    return rvec, tvec


def compute_relative_pose(
    corners,
    ids: np.ndarray,
    marker_size: float,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    fixed_id: int,
    mobile_id: int,
) -> Optional[DetectionResult]:
    ids_flat = ids.flatten().tolist()
    if fixed_id not in ids_flat or mobile_id not in ids_flat:
        return None

    fixed_index = ids_flat.index(fixed_id)
    mobile_index = ids_flat.index(mobile_id)

    fixed_rvec, fixed_tvec = estimate_single_marker_pose(
        corners[fixed_index], marker_size, camera_matrix, dist_coeffs
    )
    fixed_rotation, _ = cv2.Rodrigues(fixed_rvec)
    camera_in_fixed_rotation = fixed_rotation.T
    camera_in_fixed_translation = -fixed_rotation.T @ fixed_tvec

    mobile_rvec, mobile_tvec = estimate_single_marker_pose(
        corners[mobile_index], marker_size, camera_matrix, dist_coeffs
    )
    mobile_rotation, _ = cv2.Rodrigues(mobile_rvec)

    mobile_in_fixed_translation = (
        camera_in_fixed_rotation @ mobile_tvec + camera_in_fixed_translation
    )
    mobile_in_fixed_rotation = camera_in_fixed_rotation @ mobile_rotation
    relative_rvec, _ = cv2.Rodrigues(mobile_in_fixed_rotation)
    orientation_deg = rvec_to_euler_deg(relative_rvec)

    return DetectionResult(
        position_m=mobile_in_fixed_translation.flatten(),
        orientation_deg=orientation_deg,
        fixed_rvec=fixed_rvec,
        fixed_tvec=fixed_tvec,
        mobile_rvec=mobile_rvec,
        mobile_tvec=mobile_tvec,
    )


def resolve_share_file(filename: str) -> Path:
    candidate = PACKAGE_ROOT / "share" / "stewart_control" / filename
    if candidate.is_file():
        return candidate
    raise FileNotFoundError(f"Could not find calibration file: {candidate}")


def resolve_config_path(explicit_path: Optional[Path]) -> Path:
    if explicit_path is not None:
        if explicit_path.is_file():
            return explicit_path
        raise FileNotFoundError(f"Could not find config file: {explicit_path}")

    candidate = PACKAGE_ROOT / "config" / "stewart_params.yaml"
    if candidate.is_file():
        return candidate
    raise FileNotFoundError(f"Could not find default config file: {candidate}")


def _parse_scalar(raw_value: str):
    value = raw_value.split("#", 1)[0].strip()
    if not value:
        return None

    quoted_match = re.fullmatch(r"['\"](.*)['\"]", value)
    if quoted_match:
        return quoted_match.group(1)

    number_match = re.search(r"[-+]?\d+(?:\.\d+)?", value)
    if number_match and number_match.group(0) == value:
        if "." in value:
            return float(value)
        return int(value)

    if number_match:
        token = number_match.group(0)
        if "." in token:
            return float(token)
        return int(token)

    return value


def load_aruco_config(config_path: Path) -> dict[str, object]:
    config = dict(DEFAULT_ARUCO_CONFIG)
    in_aruco_block = False

    for raw_line in config_path.read_text(encoding="utf-8").splitlines():
        line = raw_line.rstrip()
        stripped = line.strip()

        if not stripped or stripped.startswith("#"):
            continue

        if not in_aruco_block:
            if stripped == "aruco:":
                in_aruco_block = True
            continue

        if not line.startswith("  "):
            break

        key_value = stripped.split(":", 1)
        if len(key_value) != 2:
            continue

        key, raw_value = key_value
        key = key.strip()
        if key not in config:
            continue

        parsed_value = _parse_scalar(raw_value)
        if parsed_value is not None:
            config[key] = parsed_value

    config["dictionary"] = str(config["dictionary"])
    config["marker_size"] = float(config["marker_size"])
    config["fixed_marker_id"] = int(config["fixed_marker_id"])
    config["mobile_marker_id"] = int(config["mobile_marker_id"])
    return config


def set_if_requested(cap: cv2.VideoCapture, prop_id: int, value, label: str) -> None:
    if value is None:
        return
    success = cap.set(prop_id, value)
    actual = cap.get(prop_id)
    print(f"{label:18s} request={value!r:<8} applied={success!s:<5} readback={actual}")


def apply_camera_settings(cap: cv2.VideoCapture, args: argparse.Namespace) -> None:
    set_if_requested(cap, cv2.CAP_PROP_FRAME_WIDTH, args.width, "frame_width")
    set_if_requested(cap, cv2.CAP_PROP_FRAME_HEIGHT, args.height, "frame_height")
    set_if_requested(cap, cv2.CAP_PROP_BUFFERSIZE, 1, "buffersize")
    if args.fourcc is not None:
        set_if_requested(cap, cv2.CAP_PROP_FOURCC, DEFAULT_FOURCC[args.fourcc], "fourcc")

    if args.auto_exposure is not None:
        requested = 0.75 if args.auto_exposure == "on" else 0.25
        set_if_requested(cap, cv2.CAP_PROP_AUTO_EXPOSURE, requested, "auto_exposure")

    if args.auto_wb is not None:
        requested = 1.0 if args.auto_wb == "on" else 0.0
        set_if_requested(cap, cv2.CAP_PROP_AUTO_WB, requested, "auto_white_balance")

    if args.autofocus is not None and hasattr(cv2, "CAP_PROP_AUTOFOCUS"):
        requested = 1.0 if args.autofocus == "on" else 0.0
        set_if_requested(cap, cv2.CAP_PROP_AUTOFOCUS, requested, "autofocus")

    if args.focus is not None and hasattr(cv2, "CAP_PROP_FOCUS"):
        set_if_requested(cap, cv2.CAP_PROP_FOCUS, args.focus, "focus")

    set_if_requested(cap, cv2.CAP_PROP_EXPOSURE, args.exposure, "exposure")
    set_if_requested(cap, cv2.CAP_PROP_GAIN, args.gain, "gain")
    set_if_requested(cap, cv2.CAP_PROP_BRIGHTNESS, args.brightness, "brightness")
    set_if_requested(cap, cv2.CAP_PROP_CONTRAST, args.contrast, "contrast")
    set_if_requested(cap, cv2.CAP_PROP_SATURATION, args.saturation, "saturation")
    set_if_requested(cap, cv2.CAP_PROP_SHARPNESS, args.sharpness, "sharpness")
    set_if_requested(cap, cv2.CAP_PROP_WB_TEMPERATURE, args.wb_temperature, "wb_temperature")


def print_camera_properties(cap: cv2.VideoCapture) -> None:
    props = [
        ("frame_width", cv2.CAP_PROP_FRAME_WIDTH),
        ("frame_height", cv2.CAP_PROP_FRAME_HEIGHT),
        ("fps", cv2.CAP_PROP_FPS),
        ("buffersize", cv2.CAP_PROP_BUFFERSIZE),
        ("fourcc", cv2.CAP_PROP_FOURCC),
        ("auto_exposure", cv2.CAP_PROP_AUTO_EXPOSURE),
        ("exposure", cv2.CAP_PROP_EXPOSURE),
        ("gain", cv2.CAP_PROP_GAIN),
        ("brightness", cv2.CAP_PROP_BRIGHTNESS),
        ("contrast", cv2.CAP_PROP_CONTRAST),
        ("saturation", cv2.CAP_PROP_SATURATION),
        ("sharpness", cv2.CAP_PROP_SHARPNESS),
        ("auto_wb", cv2.CAP_PROP_AUTO_WB),
        ("wb_temperature", cv2.CAP_PROP_WB_TEMPERATURE),
    ]
    if hasattr(cv2, "CAP_PROP_AUTOFOCUS"):
        props.append(("autofocus", cv2.CAP_PROP_AUTOFOCUS))
    if hasattr(cv2, "CAP_PROP_FOCUS"):
        props.append(("focus", cv2.CAP_PROP_FOCUS))

    print()
    print("Camera properties:")
    for label, prop_id in props:
        value = cap.get(prop_id)
        if label == "fourcc":
            int_value = int(value)
            decoded = "".join(chr((int_value >> (8 * i)) & 0xFF) for i in range(4))
            print(f"  {label:16s} {value} ({decoded})")
        else:
            print(f"  {label:16s} {value}")
    print()


def frame_stats(frame: np.ndarray) -> dict[str, float]:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return {
        "mean": float(np.mean(gray)),
        "p99": float(np.percentile(gray, 99)),
        "sharpness": float(cv2.Laplacian(gray, cv2.CV_64F).var()),
    }


def smooth_detection_result(
    previous: Optional[SmoothedPose],
    current: DetectionResult,
    alpha: float,
) -> SmoothedPose:
    current_position = np.asarray(current.position_m, dtype=np.float64)
    current_orientation = np.asarray(current.orientation_deg, dtype=np.float64)

    if previous is None:
        return SmoothedPose(
            position_m=current_position,
            orientation_deg=tuple(current_orientation.tolist()),
            stale_frames=0,
        )

    previous_position = np.asarray(previous.position_m, dtype=np.float64)
    previous_orientation = np.asarray(previous.orientation_deg, dtype=np.float64)

    smoothed_position = ((1.0 - alpha) * previous_position) + (alpha * current_position)
    smoothed_orientation = ((1.0 - alpha) * previous_orientation) + (
        alpha * current_orientation
    )
    return SmoothedPose(
        position_m=smoothed_position,
        orientation_deg=tuple(smoothed_orientation.tolist()),
        stale_frames=0,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Standalone test for dual ArUco detection and relative pose."
    )
    parser.add_argument("--camera", type=int, default=0, help="OpenCV camera index.")
    parser.add_argument(
        "--backend",
        choices=sorted(DEFAULT_BACKEND.keys()),
        default="dshow",
        help="OpenCV backend. On Windows, DSHOW often exposes camera controls better.",
    )
    parser.add_argument("--width", type=int, default=640, help="Requested capture width.")
    parser.add_argument("--height", type=int, default=480, help="Requested capture height.")
    parser.add_argument(
        "--fourcc",
        choices=sorted(DEFAULT_FOURCC.keys()),
        default=None,
        help="Requested pixel format / codec.",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=None,
        help="Optional path to the Stewart YAML config.",
    )
    parser.add_argument(
        "--intrinsics",
        type=Path,
        default=None,
        help="Optional path to calib_int.npz.",
    )
    parser.add_argument(
        "--show-rejected",
        action="store_true",
        help="Draw rejected candidates in purple for debugging.",
    )
    parser.add_argument(
        "--window-name",
        default="ArUco Relative Pose Test",
        help="Preview window title.",
    )
    # Leave camera controls unset by default so the script does not clobber
    # platform-specific settings that were already applied with tools like
    # v4l2-ctl on Linux.
    parser.add_argument("--auto-exposure", choices=["on", "off"], default=None)
    parser.add_argument("--exposure", type=float, default=None)
    parser.add_argument("--gain", type=float, default=None)
    parser.add_argument("--brightness", type=float, default=None)
    parser.add_argument("--contrast", type=float, default=None)
    parser.add_argument("--saturation", type=float, default=None)
    parser.add_argument("--sharpness", type=float, default=None)
    parser.add_argument("--auto-wb", choices=["on", "off"], default=None)
    parser.add_argument("--wb-temperature", type=float, default=None)
    parser.add_argument("--autofocus", choices=["on", "off"], default=None)
    parser.add_argument("--focus", type=float, default=None)
    parser.add_argument(
        "--pose-smoothing",
        type=float,
        default=0.35,
        help="EMA factor for displayed pose smoothing. Higher tracks motion faster.",
    )
    parser.add_argument(
        "--hold-last-frames",
        type=int,
        default=4,
        help="Keep the last good pose this many frames after brief marker loss.",
    )
    return parser.parse_args()


def load_intrinsics(intrinsics_path: Path) -> tuple[np.ndarray, np.ndarray]:
    intrinsics = np.load(intrinsics_path)
    return intrinsics["mtx"], intrinsics["dist"]


def main() -> int:
    args = parse_args()
    config_path = resolve_config_path(args.config)
    aruco_cfg = load_aruco_config(config_path)

    dictionary_id = getattr(cv2.aruco, aruco_cfg["dictionary"])
    aruco_dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    detector_parameters = create_detector_parameters()

    intrinsics_path = args.intrinsics if args.intrinsics is not None else resolve_share_file("calib_int.npz")
    camera_matrix, dist_coeffs = load_intrinsics(intrinsics_path)

    marker_size = float(aruco_cfg["marker_size"])
    fixed_id = int(aruco_cfg["fixed_marker_id"])
    mobile_id = int(aruco_cfg["mobile_marker_id"])

    backend = DEFAULT_BACKEND[args.backend]
    cap = cv2.VideoCapture(args.camera, backend)
    if not cap.isOpened():
        print(
            f"Could not open camera index {args.camera} with backend {args.backend}.",
            file=sys.stderr,
        )
        return 1

    apply_camera_settings(cap, args)
    print_camera_properties(cap)

    print("ArUco runtime test started.")
    print(f"Dictionary: {aruco_cfg['dictionary']}")
    print(f"Fixed marker id: {fixed_id}")
    print(f"Mobile marker id: {mobile_id}")
    print(f"Marker size: {marker_size:.6f} m")
    print(f"Config: {config_path}")
    print(f"Intrinsics: {intrinsics_path}")
    print("Press P to print camera properties. Press Q or Esc to quit.")

    last_print_time = 0.0
    smoothed_pose: Optional[SmoothedPose] = None

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Camera frame read failed.", file=sys.stderr)
                return 1

            stats = frame_stats(frame)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = detect_markers(
                gray, aruco_dictionary, detector_parameters
            )

            status = "Waiting for both configured markers"
            pose_to_display: Optional[SmoothedPose] = None

            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                if args.show_rejected and rejected:
                    cv2.aruco.drawDetectedMarkers(
                        frame,
                        rejected,
                        borderColor=(255, 0, 255),
                    )

                result = compute_relative_pose(
                    corners=corners,
                    ids=ids,
                    marker_size=marker_size,
                    camera_matrix=camera_matrix,
                    dist_coeffs=dist_coeffs,
                    fixed_id=fixed_id,
                    mobile_id=mobile_id,
                )

                ids_flat = ids.flatten().tolist()
                if fixed_id in ids_flat:
                    fixed_index = ids_flat.index(fixed_id)
                    fixed_rvec, fixed_tvec = estimate_single_marker_pose(
                        corners[fixed_index], marker_size, camera_matrix, dist_coeffs
                    )
                    draw_axes(frame, camera_matrix, dist_coeffs, fixed_rvec, fixed_tvec)

                if mobile_id in ids_flat:
                    mobile_index = ids_flat.index(mobile_id)
                    mobile_rvec, mobile_tvec = estimate_single_marker_pose(
                        corners[mobile_index], marker_size, camera_matrix, dist_coeffs
                    )
                    draw_axes(frame, camera_matrix, dist_coeffs, mobile_rvec, mobile_tvec)

                if result is not None:
                    smoothed_pose = smooth_detection_result(
                        smoothed_pose,
                        result,
                        alpha=max(0.0, min(1.0, args.pose_smoothing)),
                    )
                    pose_to_display = smoothed_pose
                    x_m, y_m, z_m = pose_to_display.position_m
                    roll_deg, pitch_deg, yaw_deg = pose_to_display.orientation_deg

                    now = time.time()
                    if now - last_print_time >= 0.5:
                        print(
                            "position_m="
                            f"[{x_m:+.5f}, {y_m:+.5f}, {z_m:+.5f}] "
                            "orientation_deg="
                            f"[{roll_deg:+.2f}, {pitch_deg:+.2f}, {yaw_deg:+.2f}]"
                        )
                        last_print_time = now
                else:
                    missing = [
                        marker_id
                        for marker_id in (fixed_id, mobile_id)
                        if marker_id not in ids_flat
                    ]
                    status = f"Detected ids: {ids_flat} | missing: {missing}"
            if pose_to_display is None and smoothed_pose is not None:
                if smoothed_pose.stale_frames < args.hold_last_frames:
                    smoothed_pose = SmoothedPose(
                        position_m=smoothed_pose.position_m,
                        orientation_deg=smoothed_pose.orientation_deg,
                        stale_frames=smoothed_pose.stale_frames + 1,
                    )
                    pose_to_display = smoothed_pose
                    status = f"Holding last pose ({smoothed_pose.stale_frames}/{args.hold_last_frames})"
                else:
                    smoothed_pose = None

            if pose_to_display is not None:
                x_m, y_m, z_m = pose_to_display.position_m
                roll_deg, pitch_deg, yaw_deg = pose_to_display.orientation_deg
                color = (0, 255, 255) if pose_to_display.stale_frames == 0 else (0, 180, 255)
                cv2.putText(
                    frame,
                    (
                        f"Mobile in fixed frame | X={x_m:+.4f} m "
                        f"Y={y_m:+.4f} m Z={z_m:+.4f} m"
                    ),
                    (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    color,
                    2,
                )
                cv2.putText(
                    frame,
                    f"R={roll_deg:+.1f} deg  P={pitch_deg:+.1f} deg  Y={yaw_deg:+.1f} deg",
                    (20, 65),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 255, 100) if pose_to_display.stale_frames == 0 else (0, 200, 200),
                    2,
                )

            cv2.putText(
                frame,
                status,
                (20, 95),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 0, 0),
                2,
            )
            cv2.putText(
                frame,
                (
                    f"mean={stats['mean']:.1f} p99={stats['p99']:.1f} "
                    f"sharpness={stats['sharpness']:.1f}"
                ),
                (20, 120),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 0, 0),
                2,
            )
            cv2.imshow(args.window_name, frame)

            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
            if key in (ord("p"), ord("P")):
                print_camera_properties(cap)
                print(
                    "Current frame stats: "
                    f"mean={stats['mean']:.1f} p99={stats['p99']:.1f} "
                    f"sharpness={stats['sharpness']:.1f}"
                )
    finally:
        cap.release()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
