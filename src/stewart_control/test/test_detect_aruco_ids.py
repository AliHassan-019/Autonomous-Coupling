#!/usr/bin/env python3
"""
Live ArUco marker ID detector.

Examples:
  python3 test_detect_aruco_ids.py
  python3 test_detect_aruco_ids.py --camera 0 --backend v4l2
  python3 test_detect_aruco_ids.py --dictionary DICT_4X4_1000 --headless
  python3 test_detect_aruco_ids.py --scan-dictionaries
"""

from __future__ import annotations

import argparse
import time

import cv2
import numpy as np


BACKENDS = {
    "any": cv2.CAP_ANY,
    "dshow": getattr(cv2, "CAP_DSHOW", cv2.CAP_ANY),
    "msmf": getattr(cv2, "CAP_MSMF", cv2.CAP_ANY),
    "v4l2": getattr(cv2, "CAP_V4L2", cv2.CAP_ANY),
}

COMMON_DICTIONARIES = [
    "DICT_4X4_50",
    "DICT_4X4_100",
    "DICT_4X4_250",
    "DICT_4X4_1000",
    "DICT_5X5_50",
    "DICT_5X5_100",
    "DICT_5X5_250",
    "DICT_5X5_1000",
    "DICT_6X6_50",
    "DICT_6X6_100",
    "DICT_6X6_250",
    "DICT_6X6_1000",
    "DICT_7X7_50",
    "DICT_7X7_100",
    "DICT_7X7_250",
    "DICT_7X7_1000",
    "DICT_ARUCO_ORIGINAL",
]


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


def marker_dimensions_px(marker_corners: np.ndarray) -> tuple[float, float, float]:
    points = marker_corners.reshape(4, 2).astype(np.float64)
    top = np.linalg.norm(points[1] - points[0])
    right = np.linalg.norm(points[2] - points[1])
    bottom = np.linalg.norm(points[3] - points[2])
    left = np.linalg.norm(points[0] - points[3])
    width = (top + bottom) / 2.0
    height = (left + right) / 2.0
    area = float(cv2.contourArea(points.astype(np.float32)))
    return width, height, area


def family_from_dictionary_name(dictionary_name: str) -> str:
    if dictionary_name == "DICT_ARUCO_ORIGINAL":
        return "ARUCO_ORIGINAL"
    parts = dictionary_name.split("_")
    return parts[1] if len(parts) >= 2 else dictionary_name


def build_dictionary_list(args: argparse.Namespace):
    names = COMMON_DICTIONARIES if args.scan_dictionaries else [args.dictionary]
    dictionaries = []
    for name in names:
        dictionary_id = getattr(cv2.aruco, name, None)
        if dictionary_id is None:
            if not args.scan_dictionaries:
                raise ValueError(f"Unknown ArUco dictionary: {name}")
            continue
        dictionaries.append((name, cv2.aruco.getPredefinedDictionary(dictionary_id)))
    return dictionaries


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read the camera and print the IDs of detected ArUco markers."
    )
    parser.add_argument("--camera", type=int, default=0, help="OpenCV camera index.")
    parser.add_argument(
        "--backend",
        choices=sorted(BACKENDS.keys()),
        default="any",
        help="OpenCV camera backend. Use v4l2 on Linux if the default is unreliable.",
    )
    parser.add_argument("--width", type=int, default=640, help="Requested capture width.")
    parser.add_argument("--height", type=int, default=480, help="Requested capture height.")
    parser.add_argument(
        "--dictionary",
        default="DICT_4X4_1000",
        help="ArUco dictionary name, for example DICT_4X4_50 or DICT_4X4_1000.",
    )
    parser.add_argument(
        "--scan-dictionaries",
        action="store_true",
        help="Try common ArUco dictionaries to help identify the marker family.",
    )
    parser.add_argument(
        "--print-every",
        type=float,
        default=0.5,
        help="Minimum seconds between repeated ID prints.",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Print IDs without opening a preview window.",
    )
    return parser.parse_args()


def draw_detected_ids(
    frame: np.ndarray,
    detections: list[tuple[str, np.ndarray, np.ndarray]],
) -> np.ndarray:
    output = frame.copy()
    if not detections:
        cv2.putText(
            output,
            "No ArUco marker detected",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )
        return output

    for dictionary_name, corners, ids in detections:
        cv2.aruco.drawDetectedMarkers(output, corners, ids)
        for marker_corners, marker_id in zip(corners, ids.flatten()):
            points = marker_corners.reshape(4, 2)
            center = points.mean(axis=0).astype(int)
            width_px, height_px, _ = marker_dimensions_px(marker_corners)
            label = f"{dictionary_name} ID {int(marker_id)} {width_px:.0f}x{height_px:.0f}px"
            cv2.putText(
                output,
                label,
                (int(center[0]) - 90, int(center[1])),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
    return output


def main() -> int:
    args = parse_args()

    if not hasattr(cv2, "aruco"):
        print("Your OpenCV install does not include cv2.aruco. Install opencv-contrib-python.")
        return 1

    try:
        dictionaries = build_dictionary_list(args)
    except ValueError as exc:
        print(exc)
        return 1
    parameters = create_detector_parameters()

    cap = cv2.VideoCapture(args.camera, BACKENDS[args.backend])
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print(f"Could not open camera index {args.camera} with backend {args.backend}.")
        return 1

    print("ArUco ID detector started.")
    print(f"Camera: {args.camera} ({args.backend})")
    if args.scan_dictionaries:
        print("Dictionary scan: enabled")
        print("Note: the same visual marker may occasionally match more than one dictionary.")
    else:
        print(f"Dictionary: {args.dictionary}")
    print("Press Q or Esc in the preview window to quit.")

    last_print_time = 0.0
    last_detection_summary: tuple[tuple[str, int, int, int], ...] = ()

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Camera frame read failed.")
                return 1

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detections = []
            for dictionary_name, dictionary in dictionaries:
                corners, ids, _ = detect_markers(gray, dictionary, parameters)
                if ids is not None:
                    detections.append((dictionary_name, corners, ids))

            detection_summary = tuple(
                sorted(
                    (
                        dictionary_name,
                        int(marker_id),
                        int(round(marker_dimensions_px(marker_corners)[0])),
                        int(round(marker_dimensions_px(marker_corners)[1])),
                    )
                    for dictionary_name, corners, ids in detections
                    for marker_corners, marker_id in zip(corners, ids.flatten())
                )
            )

            now = time.monotonic()
            should_print = (
                detection_summary != last_detection_summary
                or now - last_print_time >= args.print_every
            )
            if should_print:
                if detection_summary:
                    print("Detected markers:")
                    for dictionary_name, marker_id, width_px, height_px in detection_summary:
                        family = family_from_dictionary_name(dictionary_name)
                        print(
                            f"  dictionary={dictionary_name:<18s} "
                            f"family={family:<8s} id={marker_id:<4d} "
                            f"size_px={width_px}x{height_px}"
                        )
                else:
                    print("Detected markers: none")
                last_detection_summary = detection_summary
                last_print_time = now

            if not args.headless:
                preview = draw_detected_ids(frame, detections)
                cv2.imshow("ArUco ID Detector", preview)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break

    finally:
        cap.release()
        if not args.headless:
            cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
