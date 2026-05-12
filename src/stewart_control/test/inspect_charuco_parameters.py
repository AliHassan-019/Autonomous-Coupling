#!/usr/bin/env python3

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Sequence

import cv2
import numpy as np


DICT_NAMES = [
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
]


@dataclass
class CandidateResult:
    dictionary: str
    squares_x: int
    squares_y: int
    marker_count: int
    charuco_count: int
    score: float


def _create_detector_parameters():
    if hasattr(cv2.aruco, "DetectorParameters"):
        return cv2.aruco.DetectorParameters()
    return cv2.aruco.DetectorParameters_create()


def _build_charuco_board(
    squares_x: int,
    squares_y: int,
    square_length: float,
    marker_length: float,
    dictionary,
):
    if hasattr(cv2.aruco, "CharucoBoard"):
        return cv2.aruco.CharucoBoard(
            (squares_x, squares_y),
            square_length,
            marker_length,
            dictionary,
        )
    return cv2.aruco.CharucoBoard_create(
        squares_x,
        squares_y,
        square_length,
        marker_length,
        dictionary,
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


def _load_image(image_path: Optional[Path], camera: Optional[int]) -> np.ndarray:
    if image_path is not None:
        frame = cv2.imread(str(image_path))
        if frame is None:
            raise FileNotFoundError(f"Could not read image: {image_path}")
        return frame

    cap = cv2.VideoCapture(camera if camera is not None else 0)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera index {camera if camera is not None else 0}")

    print("Press SPACE to capture a frame, or Q/Esc to quit.")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError("Camera frame read failed.")

            cv2.putText(
                frame,
                "SPACE capture | Q quit",
                (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 255),
                2,
            )
            cv2.imshow("ChArUco Inspector", frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                raise KeyboardInterrupt
            if key == ord(" "):
                return frame.copy()
    finally:
        cap.release()
        cv2.destroyAllWindows()


def inspect_frame(
    frame: np.ndarray,
    dictionary_names: Iterable[str],
    min_squares: int,
    max_squares: int,
    marker_square_ratio: float,
    top_k: int,
) -> List[CandidateResult]:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    parameters = _create_detector_parameters()
    results: List[CandidateResult] = []

    for dict_name in dictionary_names:
        if not hasattr(cv2.aruco, dict_name):
            continue

        dictionary_id = getattr(cv2.aruco, dict_name)
        dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        marker_corners, marker_ids, _ = _detect_markers(gray, dictionary, parameters)

        if marker_ids is None or len(marker_ids) == 0:
            continue

        marker_count = int(len(marker_ids))
        square_length = 1.0
        marker_length = marker_square_ratio * square_length

        for squares_x in range(min_squares, max_squares + 1):
            for squares_y in range(min_squares, max_squares + 1):
                board = _build_charuco_board(
                    squares_x,
                    squares_y,
                    square_length,
                    marker_length,
                    dictionary,
                )
                _, charuco_corners, charuco_ids = _interpolate_charuco(
                    marker_corners,
                    marker_ids,
                    gray,
                    board,
                )
                charuco_count = 0 if charuco_ids is None else int(len(charuco_ids))
                if charuco_count == 0:
                    continue

                # Favor solutions with many interpolated corners and many markers.
                score = (charuco_count * 10.0) + marker_count
                results.append(
                    CandidateResult(
                        dictionary=dict_name,
                        squares_x=squares_x,
                        squares_y=squares_y,
                        marker_count=marker_count,
                        charuco_count=charuco_count,
                        score=score,
                    )
                )

    results.sort(
        key=lambda item: (item.score, item.charuco_count, item.marker_count),
        reverse=True,
    )
    return results[:top_k]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Inspect a ChArUco board image and suggest likely parameters."
    )
    parser.add_argument(
        "--image",
        type=Path,
        help="Path to an image of the board. If omitted, captures one frame from the camera.",
    )
    parser.add_argument(
        "--camera",
        type=int,
        default=0,
        help="Camera index used when --image is not provided.",
    )
    parser.add_argument(
        "--min-squares",
        type=int,
        default=4,
        help="Minimum board size to test in each axis.",
    )
    parser.add_argument(
        "--max-squares",
        type=int,
        default=14,
        help="Maximum board size to test in each axis.",
    )
    parser.add_argument(
        "--marker-size-mm",
        type=float,
        default=11.0,
        help="Measured marker size in mm. Used only for the ratio check.",
    )
    parser.add_argument(
        "--square-size-mm",
        type=float,
        default=15.0,
        help="Measured checker square size in mm. Used only for the ratio check.",
    )
    parser.add_argument(
        "--top-k",
        type=int,
        default=10,
        help="How many candidate parameter sets to print.",
    )
    parser.add_argument(
        "--family",
        choices=["all", "4x4", "5x5", "6x6", "7x7"],
        default="all",
        help="Restrict the dictionary family to test.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        frame = _load_image(args.image, args.camera)
    except KeyboardInterrupt:
        print("Cancelled.")
        return 0
    except Exception as exc:
        print(str(exc))
        return 1

    ratio = args.marker_size_mm / args.square_size_mm
    if not (0.0 < ratio < 1.0):
        print("marker-size-mm must be smaller than square-size-mm.")
        return 1

    if args.family == "all":
        dictionary_names = DICT_NAMES
    else:
        prefix = f"DICT_{args.family.upper()}"
        dictionary_names = [name for name in DICT_NAMES if name.startswith(prefix)]

    results = inspect_frame(
        frame=frame,
        dictionary_names=dictionary_names,
        min_squares=args.min_squares,
        max_squares=args.max_squares,
        marker_square_ratio=ratio,
        top_k=args.top_k,
    )

    if not results:
        print("No plausible ChArUco parameter set found.")
        print("Check that the image shows a real ChArUco board clearly and mostly fully.")
        return 1

    print()
    print("Top candidate parameter sets:")
    for idx, item in enumerate(results, start=1):
        print(
            f"{idx:2d}. dict={item.dictionary} "
            f"squares_x={item.squares_x} squares_y={item.squares_y} "
            f"markers={item.marker_count} charuco={item.charuco_count} "
            f"score={item.score:.1f}"
        )

    print()
    print("Notes:")
    print("- The best candidate should usually have the highest charuco corner count.")
    print("- Physical square/marker size must still be measured manually.")
    print("- If you already know the board family is 4x4, re-run with --family 4x4.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
