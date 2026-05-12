#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
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


@dataclass
class CalibrationSample:
    charuco_corners: np.ndarray
    charuco_ids: np.ndarray
    image_size: Tuple[int, int]
    sharpness: float
    marker_count: int
    charuco_count: int
    coverage_cell: Tuple[int, int]
    capture_time: float


def detect_charuco_sample(
    frame: np.ndarray,
    dictionary,
    board,
    parameters,
    min_markers: int,
    min_charuco: int,
    grid_cols: int,
    grid_rows: int,
) -> Tuple[Optional[CalibrationSample], np.ndarray]:
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
    sample = CalibrationSample(
        charuco_corners=charuco_corners.copy(),
        charuco_ids=charuco_ids.copy(),
        image_size=(width, height),
        sharpness=_sharpness_score(gray),
        marker_count=int(len(marker_ids)),
        charuco_count=int(len(charuco_ids)),
        coverage_cell=_coverage_cell(
            charuco_corners, width, height, grid_cols=grid_cols, grid_rows=grid_rows
        ),
        capture_time=time.time(),
    )
    return sample, preview


def sample_is_diverse(
    sample: CalibrationSample,
    accepted: Sequence[CalibrationSample],
    min_time_delta_s: float,
    max_per_cell: int,
) -> bool:
    cell_count = sum(1 for item in accepted if item.coverage_cell == sample.coverage_cell)
    if cell_count >= max_per_cell:
        return False

    if not accepted:
        return True

    latest = accepted[-1]
    if sample.capture_time - latest.capture_time < min_time_delta_s:
        return False

    return True


def calibrate_charuco(
    samples: Sequence[CalibrationSample],
    board,
    image_size: Tuple[int, int],
) -> dict:
    all_corners = [sample.charuco_corners for sample in samples]
    all_ids = [sample.charuco_ids for sample in samples]

    flags = (
        cv2.CALIB_RATIONAL_MODEL
        | cv2.CALIB_FIX_S1_S2_S3_S4
        | cv2.CALIB_FIX_TAUX_TAUY
    )
    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        200,
        1e-9,
    )

    extended_fn = getattr(cv2.aruco, "calibrateCameraCharucoExtended", None)
    if extended_fn is not None:
        (
            rms,
            camera_matrix,
            dist_coeffs,
            rvecs,
            tvecs,
            std_intrinsics,
            std_extrinsics,
            per_view_errors,
        ) = extended_fn(
            charucoCorners=all_corners,
            charucoIds=all_ids,
            board=board,
            imageSize=image_size,
            cameraMatrix=None,
            distCoeffs=None,
            flags=flags,
            criteria=criteria,
        )
        return {
            "rms": float(rms),
            "mtx": camera_matrix,
            "dist": dist_coeffs,
            "rvecs": rvecs,
            "tvecs": tvecs,
            "std_intrinsics": std_intrinsics,
            "std_extrinsics": std_extrinsics,
            "per_view_errors": np.asarray(per_view_errors).reshape(-1),
        }

    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
        charucoCorners=all_corners,
        charucoIds=all_ids,
        board=board,
        imageSize=image_size,
        cameraMatrix=None,
        distCoeffs=None,
        flags=flags,
        criteria=criteria,
    )
    return {
        "rms": float(rms),
        "mtx": camera_matrix,
        "dist": dist_coeffs,
        "rvecs": rvecs,
        "tvecs": tvecs,
        "std_intrinsics": None,
        "std_extrinsics": None,
        "per_view_errors": None,
    }


def reject_outliers(
    samples: Sequence[CalibrationSample],
    calibration: dict,
    max_view_error_px: float,
    trim_fraction: float,
) -> List[CalibrationSample]:
    per_view_errors = calibration.get("per_view_errors")
    if per_view_errors is None or len(samples) < 8:
        return list(samples)

    indexed = list(enumerate(zip(samples, per_view_errors)))
    kept = [item for item in indexed if float(item[1][1]) <= max_view_error_px]
    if len(kept) < max(8, int(len(samples) * 0.6)):
        kept = indexed

    trimmed_count = int(len(kept) * trim_fraction)
    if trimmed_count > 0 and len(kept) - trimmed_count >= 8:
        kept = sorted(kept, key=lambda item: float(item[1][1]))[:-trimmed_count]

    kept_indices = {idx for idx, _ in kept}
    return [sample for idx, sample in enumerate(samples) if idx in kept_indices]


def save_calibration(
    output_npz: Path,
    metadata_json: Path,
    calibration: dict,
    samples: Sequence[CalibrationSample],
    args: argparse.Namespace,
) -> None:
    output_npz.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        output_npz,
        mtx=calibration["mtx"],
        dist=calibration["dist"],
        rvecs=np.asarray(calibration["rvecs"], dtype=np.float64),
        tvecs=np.asarray(calibration["tvecs"], dtype=np.float64),
    )

    metadata = {
        "created_at_epoch_s": time.time(),
        "camera_index": args.camera,
        "resolution": [args.width, args.height],
        "dictionary": args.dictionary,
        "board": {
            "squares_x": args.squares_x,
            "squares_y": args.squares_y,
            "square_length_m": args.square_length_m,
            "marker_length_m": args.marker_length_m,
        },
        "capture": {
            "min_markers": args.min_markers,
            "min_charuco": args.min_charuco,
            "min_sharpness": args.min_sharpness,
            "grid_cols": args.grid_cols,
            "grid_rows": args.grid_rows,
            "max_per_cell": args.max_per_cell,
        },
        "results": {
            "rms_reprojection_error_px": calibration["rms"],
            "accepted_views": len(samples),
        },
    }
    metadata_json.write_text(json.dumps(metadata, indent=2), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    repo_root = Path(__file__).resolve().parents[3]
    default_npz = (
        repo_root / "src" / "stewart_control" / "share" / "stewart_control" / "calib_int.npz"
    )
    default_meta = (
        repo_root / "src" / "stewart_control" / "share" / "stewart_control" / "calib_int_meta.json"
    )

    parser = argparse.ArgumentParser(
        description="Robust ChArUco intrinsic calibration for the Stewart camera."
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
    parser.add_argument(
        "--dictionary",
        default="DICT_4X4_1000",
        help="OpenCV ArUco dictionary name. Match your project markers if possible.",
    )
    parser.add_argument("--squares-x", type=int, default=6, help="ChArUco squares in X.")
    parser.add_argument("--squares-y", type=int, default=9, help="ChArUco squares in Y.")
    parser.add_argument(
        "--square-length-m",
        type=float,
        default=0.03,
        help="ChArUco square side length in meters.",
    )
    parser.add_argument(
        "--marker-length-m",
        type=float,
        default=0.024,
        help="ChArUco marker side length in meters.",
    )
    parser.add_argument(
        "--target-samples",
        type=int,
        default=35,
        help="Stop hint once this many accepted views are collected.",
    )
    parser.add_argument(
        "--min-markers",
        type=int,
        default=6,
        help="Minimum detected ArUco markers before accepting a frame.",
    )
    parser.add_argument(
        "--min-charuco",
        type=int,
        default=12,
        help="Minimum interpolated ChArUco corners before accepting a frame.",
    )
    parser.add_argument(
        "--min-sharpness",
        type=float,
        default=80.0,
        help="Minimum Laplacian variance required to accept a frame.",
    )
    parser.add_argument(
        "--grid-cols",
        type=int,
        default=4,
        help="Coverage grid width used to spread samples across the image.",
    )
    parser.add_argument(
        "--grid-rows",
        type=int,
        default=3,
        help="Coverage grid height used to spread samples across the image.",
    )
    parser.add_argument(
        "--max-per-cell",
        type=int,
        default=6,
        help="Maximum accepted samples from the same image coverage cell.",
    )
    parser.add_argument(
        "--capture-interval-s",
        type=float,
        default=0.7,
        help="Minimum time between accepted views when auto-capturing.",
    )
    parser.add_argument(
        "--max-view-error-px",
        type=float,
        default=1.2,
        help="Per-view reprojection error threshold used for outlier rejection.",
    )
    parser.add_argument(
        "--trim-fraction",
        type=float,
        default=0.1,
        help="Fraction of the worst remaining views to trim after thresholding.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=default_npz,
        help="Output .npz path. Defaults to the runtime calibration file location.",
    )
    parser.add_argument(
        "--metadata",
        type=Path,
        default=default_meta,
        help="Path for metadata JSON written alongside the calibration output.",
    )
    parser.add_argument(
        "--manual",
        action="store_true",
        help="Capture only when SPACE is pressed. Default mode auto-captures good frames.",
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

    if not hasattr(cv2, "aruco"):
        print("OpenCV was built without the aruco module.", file=sys.stderr)
        return 1

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

    accepted: List[CalibrationSample] = []
    last_auto_capture = 0.0

    print("Calibration controls:")
    print("  SPACE  capture current valid frame")
    print("  C      calibrate and save")
    print("  R      reset accepted frames")
    print("  Q/ESC  quit without saving")
    print()
    print(
        "Move the ChArUco board across the full image, vary distance/angle, "
        "and keep the board sharp."
    )

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Camera frame read failed.", file=sys.stderr)
                return 1

            sample, preview = detect_charuco_sample(
                frame=frame,
                dictionary=dictionary,
                board=board,
                parameters=parameters,
                min_markers=args.min_markers,
                min_charuco=args.min_charuco,
                grid_cols=args.grid_cols,
                grid_rows=args.grid_rows,
            )

            now = time.time()
            status = "No valid ChArUco board detected"
            can_accept = False

            if sample is not None:
                can_accept = (
                    sample.sharpness >= args.min_sharpness
                    and sample_is_diverse(
                        sample,
                        accepted,
                        min_time_delta_s=args.capture_interval_s,
                        max_per_cell=args.max_per_cell,
                    )
                )

                status = (
                    f"markers={sample.marker_count} charuco={sample.charuco_count} "
                    f"sharpness={sample.sharpness:.1f} cell={sample.coverage_cell}"
                )
                if sample.sharpness < args.min_sharpness:
                    status += " | rejected: blurry"
                elif not sample_is_diverse(
                    sample,
                    accepted,
                    min_time_delta_s=args.capture_interval_s,
                    max_per_cell=args.max_per_cell,
                ):
                    status += " | rejected: low diversity"
                else:
                    status += " | ready"

                if (
                    not args.manual
                    and can_accept
                    and now - last_auto_capture >= args.capture_interval_s
                ):
                    accepted.append(sample)
                    last_auto_capture = now

            cv2.putText(
                preview,
                f"Accepted views: {len(accepted)} / {args.target_samples}",
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
                (0, 255, 255) if can_accept else (0, 128, 255),
                2,
            )
            cv2.putText(
                preview,
                "SPACE capture | C calibrate | R reset | Q quit",
                (20, preview.shape[0] - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                2,
            )

            cv2.imshow("Stewart Camera Calibration", preview)
            key = cv2.waitKey(1) & 0xFF

            if key in (27, ord("q")):
                print("Quit without saving.")
                return 0

            if key == ord("r"):
                accepted.clear()
                last_auto_capture = 0.0
                print("Accepted views cleared.")

            if key == ord(" "):
                if sample is None:
                    print("Current frame is not a valid ChArUco detection.")
                elif sample.sharpness < args.min_sharpness:
                    print(
                        f"Current frame rejected because sharpness {sample.sharpness:.1f} "
                        f"is below threshold {args.min_sharpness:.1f}."
                    )
                elif not sample_is_diverse(
                    sample,
                    accepted,
                    min_time_delta_s=args.capture_interval_s,
                    max_per_cell=args.max_per_cell,
                ):
                    print(
                        "Current frame rejected because it is too similar to "
                        "recent samples."
                    )
                else:
                    accepted.append(sample)
                    last_auto_capture = now
                    print(f"Accepted view {len(accepted)}.")

            if key == ord("c"):
                if len(accepted) < max(12, args.target_samples // 2):
                    print(
                        f"Not enough views yet: {len(accepted)} collected. "
                        f"Collect at least {max(12, args.target_samples // 2)}."
                    )
                    continue

                image_size = accepted[0].image_size
                calibration = calibrate_charuco(accepted, board, image_size)
                filtered = reject_outliers(
                    accepted,
                    calibration,
                    max_view_error_px=args.max_view_error_px,
                    trim_fraction=args.trim_fraction,
                )

                if len(filtered) != len(accepted):
                    calibration = calibrate_charuco(filtered, board, image_size)
                    accepted = filtered

                save_calibration(
                    args.output,
                    args.metadata,
                    calibration,
                    accepted,
                    args,
                )

                print()
                print("Calibration saved successfully.")
                print(f"Output:   {args.output}")
                print(f"Metadata: {args.metadata}")
                print(f"Views:    {len(accepted)}")
                print(f"RMS px:   {calibration['rms']:.4f}")
                print("Camera matrix:")
                print(calibration["mtx"])
                print("Distortion coefficients:")
                print(calibration["dist"])
                return 0

    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    raise SystemExit(main())
