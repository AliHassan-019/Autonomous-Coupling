#!/usr/bin/env python3


from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))


DEFAULT_BACKEND = {
    "any": cv2.CAP_ANY,
    "dshow": getattr(cv2, "CAP_DSHOW", cv2.CAP_ANY),
    "msmf": getattr(cv2, "CAP_MSMF", cv2.CAP_ANY),
    "v4l2": getattr(cv2, "CAP_V4L2", cv2.CAP_ANY),
}


def create_detector_parameters():
    if hasattr(cv2.aruco, "DetectorParameters"):
        parameters = cv2.aruco.DetectorParameters()
    else:
        parameters = cv2.aruco.DetectorParameters_create()

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


def apply_camera_settings(cap: cv2.VideoCapture, args: argparse.Namespace) -> None:
    for prop_id, value in (
        (cv2.CAP_PROP_FRAME_WIDTH, args.width),
        (cv2.CAP_PROP_FRAME_HEIGHT, args.height),
        (cv2.CAP_PROP_BUFFERSIZE, 1),
        (cv2.CAP_PROP_EXPOSURE, args.exposure),
        (cv2.CAP_PROP_GAIN, args.gain),
        (cv2.CAP_PROP_BRIGHTNESS, args.brightness),
        (cv2.CAP_PROP_CONTRAST, args.contrast),
        (cv2.CAP_PROP_SATURATION, args.saturation),
        (cv2.CAP_PROP_SHARPNESS, args.sharpness),
        (cv2.CAP_PROP_WB_TEMPERATURE, args.wb_temperature),
    ):
        if value is not None:
            cap.set(prop_id, float(value))

    if args.auto_exposure is not None:
        cap.set(
            cv2.CAP_PROP_AUTO_EXPOSURE,
            0.75 if args.auto_exposure == "on" else 0.25,
        )
    if args.auto_wb is not None:
        cap.set(cv2.CAP_PROP_AUTO_WB, 1.0 if args.auto_wb == "on" else 0.0)
    if args.autofocus is not None and hasattr(cv2, "CAP_PROP_AUTOFOCUS"):
        cap.set(cv2.CAP_PROP_AUTOFOCUS, 1.0 if args.autofocus == "on" else 0.0)
    if args.focus is not None and hasattr(cv2, "CAP_PROP_FOCUS"):
        cap.set(cv2.CAP_PROP_FOCUS, float(args.focus))


def rvec_to_euler_deg(rvec: np.ndarray) -> np.ndarray:
    rotation, _ = cv2.Rodrigues(rvec)
    sy = math.sqrt(rotation[0, 0] ** 2 + rotation[1, 0] ** 2)
    singular = sy < 1e-6

    if not singular:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        pitch = math.atan2(-rotation[2, 0], sy)
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = math.atan2(-rotation[1, 2], rotation[1, 1])
        pitch = math.atan2(-rotation[2, 0], sy)
        yaw = 0.0

    return np.array([math.degrees(roll), math.degrees(pitch), math.degrees(yaw)])


def load_intrinsics(path: Path) -> tuple[np.ndarray, np.ndarray]:
    intr = np.load(path)
    return np.asarray(intr["mtx"], dtype=np.float64), np.asarray(intr["dist"], dtype=np.float64)


def load_extrinsics(path: Optional[Path]) -> Optional[tuple[np.ndarray, np.ndarray]]:
    if path is None:
        return None
    extr = np.load(path)
    rvec = np.asarray(extr["rvec"], dtype=np.float64).reshape(3, 1)
    tvec = np.asarray(extr["tvec"], dtype=np.float64).reshape(3, 1)
    rotation, _ = cv2.Rodrigues(rvec)
    return rotation, tvec


def parse_args() -> argparse.Namespace:
    default_intr = PACKAGE_ROOT / "share" / "stewart_control" / "calib_int.npz"
    default_extr = PACKAGE_ROOT / "share" / "stewart_control" / "calib_ext3.npz"

    parser = argparse.ArgumentParser(
        description="Test single ArUco marker pose from a fixed camera."
    )
    parser.add_argument("--camera", type=int, default=0, help="OpenCV camera index.")
    parser.add_argument(
        "--backend",
        choices=sorted(DEFAULT_BACKEND.keys()),
        default="dshow",
        help="OpenCV backend.",
    )
    parser.add_argument(
        "--dictionary",
        default="DICT_4X4_1000",
        help="OpenCV ArUco dictionary name.",
    )
    parser.add_argument(
        "--marker-id",
        type=int,
        default=28,
        help="Marker ID attached to the moving platform.",
    )
    parser.add_argument(
        "--marker-size",
        type=float,
        default=0.022,
        help="Marker size in meters.",
    )
    parser.add_argument(
        "--intrinsics",
        type=Path,
        default=default_intr,
        help="Path to calib_int.npz.",
    )
    parser.add_argument(
        "--extrinsics",
        type=Path,
        default=default_extr,
        help="Optional path to camera-to-base extrinsics .npz.",
    )
    parser.add_argument(
        "--no-extrinsics",
        action="store_true",
        help="Ignore extrinsics and report pose only in camera frame.",
    )
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
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
        help="EMA factor for position and Euler pose.",
    )
    parser.add_argument(
        "--window-name",
        default="Single Marker Fixed Camera Test",
        help="Preview window title.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if not args.intrinsics.is_file():
        print(f"Missing intrinsics file: {args.intrinsics}", file=sys.stderr)
        return 1

    camera_matrix, dist_coeffs = load_intrinsics(args.intrinsics)
    extrinsics = None if args.no_extrinsics else load_extrinsics(args.extrinsics)

    dictionary_id = getattr(cv2.aruco, args.dictionary)
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    parameters = create_detector_parameters()

    cap = cv2.VideoCapture(args.camera, DEFAULT_BACKEND[args.backend])
    apply_camera_settings(cap, args)
    if not cap.isOpened():
        print(
            f"Could not open camera index {args.camera} with backend {args.backend}.",
            file=sys.stderr,
        )
        return 1

    print("Single-marker test started.")
    print(f"Marker id: {args.marker_id}")
    print(f"Marker size: {args.marker_size:.6f} m")
    print(f"Intrinsics: {args.intrinsics}")
    if extrinsics is None:
        print("Base-frame conversion: disabled")
    else:
        print(f"Base-frame conversion: enabled from {args.extrinsics}")
    print("Press Q or Esc to quit.")

    smoothed_tvec = None
    smoothed_euler = None
    last_print_time = 0.0

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Camera frame read failed.", file=sys.stderr)
                return 1

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detect_markers(gray, dictionary, parameters)

            image_center = (frame.shape[1] // 2, frame.shape[0] // 2)
            cv2.circle(frame, image_center, 5, (255, 255, 0), -1)
            cv2.putText(
                frame,
                "Camera optical center",
                (image_center[0] + 10, image_center[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 0),
                1,
            )

            status = f"Waiting for marker {args.marker_id}"

            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                ids_flat = ids.flatten().tolist()

                if args.marker_id in ids_flat:
                    idx = ids_flat.index(args.marker_id)
                    marker_corner = corners[idx]
                    marker_center = tuple(
                        np.round(marker_corner.reshape(-1, 2).mean(axis=0)).astype(int)
                    )

                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        [marker_corner],
                        args.marker_size,
                        camera_matrix,
                        dist_coeffs,
                    )
                    rvec = rvecs[0].reshape(3, 1)
                    tvec = tvecs[0].reshape(3, 1)
                    euler_deg = rvec_to_euler_deg(rvec)

                    alpha = max(0.0, min(1.0, args.pose_smoothing))
                    if smoothed_tvec is None:
                        smoothed_tvec = tvec.copy()
                        smoothed_euler = euler_deg.copy()
                    else:
                        smoothed_tvec = ((1.0 - alpha) * smoothed_tvec) + (alpha * tvec)
                        smoothed_euler = ((1.0 - alpha) * smoothed_euler) + (alpha * euler_deg)

                    draw_axes(frame, camera_matrix, dist_coeffs, rvec, tvec)
                    cv2.circle(frame, marker_center, 6, (0, 255, 255), -1)
                    cv2.line(frame, image_center, marker_center, (0, 255, 255), 2)

                    x_m, y_m, z_m = smoothed_tvec.flatten().tolist()
                    distance_cm = float(np.linalg.norm(smoothed_tvec) * 100.0)
                    roll_deg, pitch_deg, yaw_deg = smoothed_euler.tolist()

                    status = (
                        f"Camera -> marker center | X={x_m * 100.0:+.2f} cm "
                        f"Y={y_m * 100.0:+.2f} cm Z={z_m * 100.0:+.2f} cm "
                        f"| D={distance_cm:.2f} cm"
                    )

                    cv2.putText(
                        frame,
                        status,
                        (20, 35),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.58,
                        (0, 255, 255),
                        2,
                    )
                    cv2.putText(
                        frame,
                        f"R={roll_deg:+.1f} deg  P={pitch_deg:+.1f} deg  Y={yaw_deg:+.1f} deg",
                        (20, 65),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.58,
                        (0, 255, 120),
                        2,
                    )

                    if extrinsics is not None:
                        rot_base_from_cam, trans_base_from_cam = extrinsics
                        marker_in_base = (rot_base_from_cam @ smoothed_tvec) + trans_base_from_cam
                        bx, by, bz = marker_in_base.flatten().tolist()
                        cv2.putText(
                            frame,
                            (
                                f"Marker in base frame | X={bx * 100.0:+.2f} cm "
                                f"Y={by * 100.0:+.2f} cm Z={bz * 100.0:+.2f} cm"
                            ),
                            (20, 95),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.58,
                            (255, 220, 0),
                            2,
                        )

                    now = time.time()
                    if now - last_print_time >= 0.5:
                        print(status)
                        last_print_time = now

            cv2.putText(
                frame,
                "Line shows image-space path from camera optical center to marker center",
                (20, frame.shape[0] - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                (255, 255, 255),
                1,
            )
            cv2.imshow(args.window_name, frame)

            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
