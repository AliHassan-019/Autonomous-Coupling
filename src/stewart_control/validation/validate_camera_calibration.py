#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from stewart_control.config_loader import load_config


DEFAULT_BACKEND = {
    "any": cv2.CAP_ANY,
    "dshow": getattr(cv2, "CAP_DSHOW", cv2.CAP_ANY),
    "msmf": getattr(cv2, "CAP_MSMF", cv2.CAP_ANY),
    "v4l2": getattr(cv2, "CAP_V4L2", cv2.CAP_ANY),
}


def _create_detector_parameters():
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


def _set_if_requested(cap: cv2.VideoCapture, prop_id: int, value, label: str) -> None:
    if value is None:
        return
    success = cap.set(prop_id, value)
    actual = cap.get(prop_id)
    print(f"{label:18s} request={value!r:<8} applied={success!s:<5} readback={actual}")


def _apply_camera_settings(cap: cv2.VideoCapture, args: argparse.Namespace) -> None:
    _set_if_requested(cap, cv2.CAP_PROP_FRAME_WIDTH, args.width, "frame_width")
    _set_if_requested(cap, cv2.CAP_PROP_FRAME_HEIGHT, args.height, "frame_height")
    _set_if_requested(cap, cv2.CAP_PROP_BUFFERSIZE, 1, "buffersize")

    if args.auto_exposure is not None:
        requested = 0.75 if args.auto_exposure == "on" else 0.25
        _set_if_requested(cap, cv2.CAP_PROP_AUTO_EXPOSURE, requested, "auto_exposure")

    if args.auto_wb is not None:
        requested = 1.0 if args.auto_wb == "on" else 0.0
        _set_if_requested(cap, cv2.CAP_PROP_AUTO_WB, requested, "auto_white_balance")

    if args.autofocus is not None and hasattr(cv2, "CAP_PROP_AUTOFOCUS"):
        requested = 1.0 if args.autofocus == "on" else 0.0
        _set_if_requested(cap, cv2.CAP_PROP_AUTOFOCUS, requested, "autofocus")

    if args.focus is not None and hasattr(cv2, "CAP_PROP_FOCUS"):
        _set_if_requested(cap, cv2.CAP_PROP_FOCUS, args.focus, "focus")

    _set_if_requested(cap, cv2.CAP_PROP_EXPOSURE, args.exposure, "exposure")
    _set_if_requested(cap, cv2.CAP_PROP_GAIN, args.gain, "gain")
    _set_if_requested(cap, cv2.CAP_PROP_BRIGHTNESS, args.brightness, "brightness")
    _set_if_requested(cap, cv2.CAP_PROP_CONTRAST, args.contrast, "contrast")
    _set_if_requested(cap, cv2.CAP_PROP_SATURATION, args.saturation, "saturation")
    _set_if_requested(cap, cv2.CAP_PROP_SHARPNESS, args.sharpness, "sharpness")
    _set_if_requested(cap, cv2.CAP_PROP_WB_TEMPERATURE, args.wb_temperature, "wb_temperature")


def _print_camera_properties(cap: cv2.VideoCapture) -> None:
    props = [
        ("frame_width", cv2.CAP_PROP_FRAME_WIDTH),
        ("frame_height", cv2.CAP_PROP_FRAME_HEIGHT),
        ("fps", cv2.CAP_PROP_FPS),
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

    print("Camera properties:")
    for label, prop_id in props:
        print(f"  {label:16s} {cap.get(prop_id)}")


def _apply_config_defaults(args: argparse.Namespace) -> argparse.Namespace:
    cfg = load_config(str(args.config) if args.config is not None else None)
    aruco_cfg = cfg.get("aruco", {})
    defaults = {
        "camera": aruco_cfg.get("camera_index", 0),
        "backend": aruco_cfg.get("camera_backend", "any"),
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
    for key, value in defaults.items():
        if getattr(args, key) is None:
            setattr(args, key, value)
    return args


def _build_charuco_board(
    squares_x: int,
    squares_y: int,
    square_length_m: float,
    marker_length_m: float,
    dictionary,
):
    if hasattr(cv2.aruco, "CharucoBoard"):
        return cv2.aruco.CharucoBoard(
            (squares_x, squares_y), square_length_m, marker_length_m, dictionary
        )
    return cv2.aruco.CharucoBoard_create(
        squares_x, squares_y, square_length_m, marker_length_m, dictionary
    )


def _detect_markers(gray: np.ndarray, dictionary, parameters):
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        return detector.detectMarkers(gray)
    return cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)


def _interpolate_charuco(
    marker_corners: Sequence[np.ndarray],
    marker_ids: np.ndarray,
    gray: np.ndarray,
    board,
):
    try:
        return cv2.aruco.interpolateCornersCharuco(
            markerCorners=marker_corners,
            markerIds=marker_ids,
            image=gray,
            board=board,
        )
    except TypeError:
        return cv2.aruco.interpolateCornersCharuco(
            marker_corners,
            marker_ids,
            gray,
            board,
        )


def _draw_detected_markers(frame: np.ndarray, corners, ids) -> None:
    if ids is not None and len(ids) > 0:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)


def _draw_detected_charuco(frame: np.ndarray, corners, ids) -> None:
    if ids is not None and len(ids) > 0:
        cv2.aruco.drawDetectedCornersCharuco(frame, corners, ids, (0, 255, 0))


def _sharpness_score(gray: np.ndarray) -> float:
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def _coverage_cell(
    points: np.ndarray,
    frame_width: int,
    frame_height: int,
    grid_cols: int,
    grid_rows: int,
) -> Tuple[int, int]:
    center = points.reshape(-1, 2).mean(axis=0)
    col = min(grid_cols - 1, max(0, int(center[0] / max(1, frame_width) * grid_cols)))
    row = min(
        grid_rows - 1, max(0, int(center[1] / max(1, frame_height) * grid_rows))
    )
    return col, row


def _board_object_points(board, charuco_ids: np.ndarray) -> np.ndarray:
    if hasattr(board, "getChessboardCorners"):
        corners_3d = np.asarray(board.getChessboardCorners(), dtype=np.float32)
    else:
        corners_3d = np.asarray(board.chessboardCorners, dtype=np.float32)
    return corners_3d[charuco_ids.flatten()]


def _rvec_to_euler_deg(rvec: np.ndarray) -> np.ndarray:
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


@dataclass
class ValidationView:
    charuco_corners: np.ndarray
    charuco_ids: np.ndarray
    marker_count: int
    charuco_count: int
    image_size: Tuple[int, int]
    sharpness: float
    coverage_cell: Tuple[int, int]
    capture_time: float


@dataclass
class PoseResult:
    success: bool
    rvec: np.ndarray
    tvec: np.ndarray
    reproj_mean_px: float
    reproj_max_px: float


def detect_validation_view(
    frame: np.ndarray,
    dictionary,
    board,
    parameters,
    min_markers: int,
    min_charuco: int,
    grid_cols: int,
    grid_rows: int,
) -> Tuple[Optional[ValidationView], np.ndarray]:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    marker_corners, marker_ids, _ = _detect_markers(gray, dictionary, parameters)
    preview = frame.copy()
    _draw_detected_markers(preview, marker_corners, marker_ids)

    if marker_ids is None or len(marker_ids) < min_markers:
        return None, preview

    _, charuco_corners, charuco_ids = _interpolate_charuco(
        marker_corners, marker_ids, gray, board
    )
    _draw_detected_charuco(preview, charuco_corners, charuco_ids)

    if charuco_ids is None or len(charuco_ids) < min_charuco:
        return None, preview

    height, width = gray.shape[:2]
    view = ValidationView(
        charuco_corners=charuco_corners.copy(),
        charuco_ids=charuco_ids.copy(),
        marker_count=int(len(marker_ids)),
        charuco_count=int(len(charuco_ids)),
        image_size=(width, height),
        sharpness=_sharpness_score(gray),
        coverage_cell=_coverage_cell(
            charuco_corners, width, height, grid_cols=grid_cols, grid_rows=grid_rows
        ),
        capture_time=time.time(),
    )
    return view, preview


def estimate_pose_and_error(
    view: ValidationView,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    board,
) -> PoseResult:
    object_points = _board_object_points(board, view.charuco_ids)
    image_points = np.asarray(view.charuco_corners, dtype=np.float32).reshape(-1, 2)
    success, rvec, tvec = cv2.solvePnP(
        object_points,
        image_points,
        camera_matrix,
        dist_coeffs,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not success:
        return PoseResult(False, None, None, float("inf"), float("inf"))

    projected, _ = cv2.projectPoints(
        object_points,
        rvec,
        tvec,
        camera_matrix,
        dist_coeffs,
    )
    projected = projected.reshape(-1, 2)
    errors = np.linalg.norm(projected - image_points, axis=1)
    return PoseResult(
        True,
        rvec,
        tvec,
        float(np.mean(errors)),
        float(np.max(errors)),
    )


def summarize_views(
    views: Sequence[ValidationView],
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    board,
    grid_cols: int,
    grid_rows: int,
) -> dict:
    pose_results = [
        estimate_pose_and_error(view, camera_matrix, dist_coeffs, board) for view in views
    ]
    pose_results = [result for result in pose_results if result.success]

    reproj_mean = np.array([result.reproj_mean_px for result in pose_results])
    reproj_max = np.array([result.reproj_max_px for result in pose_results])
    sharpness = np.array([view.sharpness for view in views])
    used_cells = {view.coverage_cell for view in views}

    return {
        "valid_views": len(pose_results),
        "captured_views": len(views),
        "coverage_cells_used": len(used_cells),
        "coverage_cells_total": int(grid_cols * grid_rows),
        "sharpness_mean": float(np.mean(sharpness)) if len(sharpness) else None,
        "sharpness_min": float(np.min(sharpness)) if len(sharpness) else None,
        "reproj_mean_avg_px": float(np.mean(reproj_mean)) if len(reproj_mean) else None,
        "reproj_mean_median_px": float(np.median(reproj_mean))
        if len(reproj_mean)
        else None,
        "reproj_mean_p95_px": float(np.percentile(reproj_mean, 95))
        if len(reproj_mean)
        else None,
        "reproj_max_avg_px": float(np.mean(reproj_max)) if len(reproj_max) else None,
        "reproj_max_worst_px": float(np.max(reproj_max)) if len(reproj_max) else None,
    }


def capture_stability_series(
    cap,
    dictionary,
    board,
    parameters,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    args: argparse.Namespace,
) -> Optional[dict]:
    poses = []
    start = time.time()
    deadline = start + args.stability_timeout_s

    while len(poses) < args.stability_frames and time.time() < deadline:
        ok, frame = cap.read()
        if not ok:
            break

        view, preview = detect_validation_view(
            frame,
            dictionary,
            board,
            parameters,
            args.min_markers,
            args.min_charuco,
            args.grid_cols,
            args.grid_rows,
        )

        if view is not None and view.sharpness >= args.min_sharpness:
            pose = estimate_pose_and_error(view, camera_matrix, dist_coeffs, board)
            if pose.success:
                poses.append(pose)

        cv2.putText(
            preview,
            f"Static stability: {len(poses)} / {args.stability_frames}",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2,
        )
        cv2.putText(
            preview,
            "Hold the board still",
            (20, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
        )
        cv2.imshow("Stewart Calibration Validation", preview)
        cv2.waitKey(1)

    if not poses:
        return None

    tvecs = np.array([pose.tvec.reshape(3) for pose in poses], dtype=np.float64)
    eulers = np.array([_rvec_to_euler_deg(pose.rvec) for pose in poses], dtype=np.float64)
    reproj = np.array([pose.reproj_mean_px for pose in poses], dtype=np.float64)

    return {
        "frames_used": int(len(poses)),
        "translation_std_mm": (np.std(tvecs, axis=0) * 1000.0).tolist(),
        "translation_peak_to_peak_mm": (
            (np.max(tvecs, axis=0) - np.min(tvecs, axis=0)) * 1000.0
        ).tolist(),
        "rotation_std_deg": np.std(eulers, axis=0).tolist(),
        "rotation_peak_to_peak_deg": (np.max(eulers, axis=0) - np.min(eulers, axis=0)).tolist(),
        "reproj_mean_avg_px": float(np.mean(reproj)),
        "reproj_mean_p95_px": float(np.percentile(reproj, 95)),
    }


def classify_calibration(summary: dict, stability: Optional[dict]) -> str:
    score = 0

    median = summary.get("reproj_mean_median_px")
    p95 = summary.get("reproj_mean_p95_px")
    worst = summary.get("reproj_max_worst_px")
    coverage_used = summary.get("coverage_cells_used", 0)
    coverage_total = max(1, summary.get("coverage_cells_total", 1))

    if median is not None:
        if median <= 0.20:
            score += 2
        elif median <= 0.35:
            score += 1

    if p95 is not None:
        if p95 <= 0.45:
            score += 2
        elif p95 <= 0.75:
            score += 1

    if worst is not None:
        if worst <= 1.2:
            score += 1
        elif worst > 2.5:
            score -= 1

    if coverage_used / coverage_total >= 0.6:
        score += 1

    if stability is not None:
        xyz_std = np.max(stability["translation_std_mm"])
        rpy_std = np.max(stability["rotation_std_deg"])
        if xyz_std <= 1.0 and rpy_std <= 0.35:
            score += 2
        elif xyz_std <= 2.0 and rpy_std <= 0.8:
            score += 1

    if score >= 7:
        return "excellent"
    if score >= 4:
        return "good"
    if score >= 2:
        return "usable"
    return "needs_improvement"


def parse_args() -> argparse.Namespace:
    repo_root = Path(__file__).resolve().parents[3]
    default_npz = (
        repo_root / "src" / "stewart_control" / "share" / "stewart_control" / "calib_int.npz"
    )
    default_report = (
        repo_root
        / "src"
        / "stewart_control"
        / "share"
        / "stewart_control"
        / "calib_validation_report.json"
    )

    parser = argparse.ArgumentParser(
        description="Validate a saved Stewart camera intrinsic calibration."
    )
    parser.add_argument("--config", type=Path, default=None, help="Optional path to Stewart YAML config.")
    parser.add_argument("--camera", type=int, default=None, help="OpenCV camera index.")
    parser.add_argument(
        "--backend",
        choices=sorted(DEFAULT_BACKEND.keys()),
        default=None,
        help="OpenCV backend. On Windows, DSHOW often exposes camera controls better.",
    )
    parser.add_argument("--width", type=int, default=None, help="Capture width in pixels.")
    parser.add_argument("--height", type=int, default=None, help="Capture height in pixels.")
    parser.add_argument("--calib", type=Path, default=default_npz, help="Path to calib_int.npz.")
    parser.add_argument("--report", type=Path, default=default_report, help="Validation report JSON.")
    parser.add_argument("--dictionary", default="DICT_4X4_1000")
    parser.add_argument("--squares-x", type=int, default=6)
    parser.add_argument("--squares-y", type=int, default=9)
    parser.add_argument("--square-length-m", type=float, default=0.03)
    parser.add_argument("--marker-length-m", type=float, default=0.024)
    parser.add_argument("--min-markers", type=int, default=6)
    parser.add_argument("--min-charuco", type=int, default=12)
    parser.add_argument("--min-sharpness", type=float, default=80.0)
    parser.add_argument("--grid-cols", type=int, default=4)
    parser.add_argument("--grid-rows", type=int, default=3)
    parser.add_argument("--capture-interval-s", type=float, default=0.7)
    parser.add_argument("--target-views", type=int, default=20)
    parser.add_argument("--stability-frames", type=int, default=50)
    parser.add_argument("--stability-timeout-s", type=float, default=20.0)
    parser.add_argument(
        "--manual",
        action="store_true",
        help="Manual capture only. Default auto-captures diverse valid views.",
    )
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
    return _apply_config_defaults(parser.parse_args())


def main() -> int:
    args = parse_args()

    if not args.calib.is_file():
        print(f"Calibration file not found: {args.calib}", file=sys.stderr)
        return 1

    intr = np.load(args.calib)
    if "mtx" not in intr or "dist" not in intr:
        print("Calibration file must contain 'mtx' and 'dist'.", file=sys.stderr)
        return 1
    camera_matrix = np.asarray(intr["mtx"], dtype=np.float64)
    dist_coeffs = np.asarray(intr["dist"], dtype=np.float64)

    try:
        dictionary_id = getattr(cv2.aruco, args.dictionary)
    except AttributeError:
        print(f"Unknown dictionary: {args.dictionary}", file=sys.stderr)
        return 1

    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    board = _build_charuco_board(
        args.squares_x,
        args.squares_y,
        args.square_length_m,
        args.marker_length_m,
        dictionary,
    )
    parameters = _create_detector_parameters()

    cap = cv2.VideoCapture(args.camera, DEFAULT_BACKEND[args.backend])
    _apply_camera_settings(cap, args)
    if not cap.isOpened():
        print(f"Could not open camera index {args.camera}.", file=sys.stderr)
        return 1
    _print_camera_properties(cap)

    captured: List[ValidationView] = []
    last_auto_capture = 0.0
    stability_report = None

    print("Validation controls:")
    print("  SPACE  capture current valid view")
    print("  S      run static stability test")
    print("  C      compute report and save")
    print("  R      reset captured views")
    print("  Q/ESC  quit")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Camera frame read failed.", file=sys.stderr)
                return 1

            view, preview = detect_validation_view(
                frame,
                dictionary,
                board,
                parameters,
                args.min_markers,
                args.min_charuco,
                args.grid_cols,
                args.grid_rows,
            )

            now = time.time()
            ready = view is not None and view.sharpness >= args.min_sharpness

            if ready and not args.manual and now - last_auto_capture >= args.capture_interval_s:
                captured.append(view)
                last_auto_capture = now

            status = "No valid ChArUco board detected"
            if view is not None:
                pose = estimate_pose_and_error(view, camera_matrix, dist_coeffs, board)
                if pose.success:
                    status = (
                        f"charuco={view.charuco_count} sharpness={view.sharpness:.1f} "
                        f"mean={pose.reproj_mean_px:.3f}px max={pose.reproj_max_px:.3f}px"
                    )
                else:
                    status = "Pose solve failed for current view"

            cv2.putText(
                preview,
                f"Views: {len(captured)} / {args.target_views}",
                (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
            )
            cv2.putText(
                preview,
                status,
                (20, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255) if ready else (0, 128, 255),
                2,
            )
            cv2.putText(
                preview,
                "SPACE capture | S stability | C report | R reset | Q quit",
                (20, preview.shape[0] - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                2,
            )

            cv2.imshow("Stewart Calibration Validation", preview)
            key = cv2.waitKey(1) & 0xFF

            if key in (27, ord("q")):
                return 0

            if key == ord("r"):
                captured.clear()
                stability_report = None
                last_auto_capture = 0.0
                print("Validation captures reset.")

            if key == ord(" "):
                if ready:
                    captured.append(view)
                    last_auto_capture = now
                    print(f"Captured validation view {len(captured)}.")
                else:
                    print("Current frame is not ready for validation capture.")

            if key == ord("s"):
                print("Running static stability test. Hold the board still.")
                stability_report = capture_stability_series(
                    cap,
                    dictionary,
                    board,
                    parameters,
                    camera_matrix,
                    dist_coeffs,
                    args,
                )
                if stability_report is None:
                    print("Static stability test failed to collect valid frames.")
                else:
                    print("Static stability test completed.")
                    print(
                        "Translation std mm:",
                        np.round(stability_report["translation_std_mm"], 3).tolist(),
                    )
                    print(
                        "Rotation std deg:",
                        np.round(stability_report["rotation_std_deg"], 3).tolist(),
                    )

            if key == ord("c"):
                if len(captured) < max(8, args.target_views // 2):
                    print(
                        f"Collect more validation views first. Current: {len(captured)}."
                    )
                    continue

                summary = summarize_views(
                    captured,
                    camera_matrix,
                    dist_coeffs,
                    board,
                    args.grid_cols,
                    args.grid_rows,
                )
                rating = classify_calibration(summary, stability_report)
                report = {
                    "created_at_epoch_s": time.time(),
                    "calibration_file": str(args.calib),
                    "resolution": [args.width, args.height],
                    "summary": summary,
                    "stability": stability_report,
                    "rating": rating,
                }
                args.report.parent.mkdir(parents=True, exist_ok=True)
                args.report.write_text(json.dumps(report, indent=2), encoding="utf-8")

                print()
                print("Validation report saved.")
                print(f"Report: {args.report}")
                print(f"Rating: {rating}")
                print(
                    "Reprojection median px:",
                    f"{summary['reproj_mean_median_px']:.4f}",
                )
                print(
                    "Reprojection p95 px:",
                    f"{summary['reproj_mean_p95_px']:.4f}",
                )
                print(
                    "Worst-point reprojection px:",
                    f"{summary['reproj_max_worst_px']:.4f}",
                )
                if stability_report is not None:
                    print(
                        "Static translation std mm:",
                        np.round(stability_report["translation_std_mm"], 3).tolist(),
                    )
                    print(
                        "Static rotation std deg:",
                        np.round(stability_report["rotation_std_deg"], 3).tolist(),
                    )
                return 0

    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    raise SystemExit(main())
