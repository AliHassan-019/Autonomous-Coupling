#!/usr/bin/env python3
"""
Controls:
  Q / ESC    Quit
  I          Toggle overlay
  P          Print current camera properties to the terminal
  S          Save a snapshot PNG next to this script
  v4l2-ctl -d /dev/video0 -c auto_exposure=1
v4l2-ctl -d /dev/video0 -c exposure_dynamic_framerate=0
v4l2-ctl -d /dev/video0 -c exposure_time_absolute=80
v4l2-ctl -d /dev/video0 -c gain=0
v4l2-ctl -d /dev/video0 -c white_balance_automatic=0
v4l2-ctl -d /dev/video0 -c white_balance_temperature=4600
v4l2-ctl -d /dev/video0 -c focus_automatic_continuous=0
v4l2-ctl -d /dev/video0 -c focus_absolute=50

"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2
import numpy as np


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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Show the raw camera feed and optionally apply fixed camera settings."
    )
    parser.add_argument("--camera", type=int, default=0, help="OpenCV camera index.")
    parser.add_argument(
        "--backend",
        choices=sorted(DEFAULT_BACKEND.keys()),
        default="any",
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
        "--auto-exposure",
        choices=["on", "off"],
        default=None,
        help="Try to enable or disable auto exposure.",
    )
    parser.add_argument(
        "--exposure",
        type=float,
        default=None,
        help="Requested manual exposure value.",
    )
    parser.add_argument("--gain", type=float, default=None, help="Requested gain value.")
    parser.add_argument(
        "--brightness",
        type=float,
        default=None,
        help="Requested brightness value.",
    )
    parser.add_argument("--contrast", type=float, default=None, help="Requested contrast value.")
    parser.add_argument("--saturation", type=float, default=None, help="Requested saturation value.")
    parser.add_argument(
        "--sharpness",
        type=float,
        default=None,
        help="Requested camera sharpness value if supported.",
    )
    parser.add_argument(
        "--auto-wb",
        choices=["on", "off"],
        default=None,
        help="Try to enable or disable auto white balance.",
    )
    parser.add_argument(
        "--wb-temperature",
        type=float,
        default=None,
        help="Requested white-balance temperature if supported.",
    )
    return parser.parse_args()


def _set_if_requested(cap: cv2.VideoCapture, prop_id: int, value, label: str) -> None:
    if value is None:
        return
    success = cap.set(prop_id, value)
    actual = cap.get(prop_id)
    print(f"{label:18s} request={value!r:<8} applied={success!s:<5} readback={actual}")


def apply_camera_settings(cap: cv2.VideoCapture, args: argparse.Namespace) -> None:
    _set_if_requested(cap, cv2.CAP_PROP_FRAME_WIDTH, args.width, "frame_width")
    _set_if_requested(cap, cv2.CAP_PROP_FRAME_HEIGHT, args.height, "frame_height")
    _set_if_requested(cap, cv2.CAP_PROP_BUFFERSIZE, 1, "buffersize")
    if args.fourcc is not None:
        _set_if_requested(cap, cv2.CAP_PROP_FOURCC, DEFAULT_FOURCC[args.fourcc], "fourcc")

    if args.auto_exposure is not None:
        # OpenCV camera backends do not agree on the exact values. These two
        # choices cover the common DirectShow / V4L conventions.
        requested = 0.75 if args.auto_exposure == "on" else 0.25
        _set_if_requested(cap, cv2.CAP_PROP_AUTO_EXPOSURE, requested, "auto_exposure")

    if args.auto_wb is not None:
        requested = 1.0 if args.auto_wb == "on" else 0.0
        _set_if_requested(cap, cv2.CAP_PROP_AUTO_WB, requested, "auto_white_balance")

    _set_if_requested(cap, cv2.CAP_PROP_EXPOSURE, args.exposure, "exposure")
    _set_if_requested(cap, cv2.CAP_PROP_GAIN, args.gain, "gain")
    _set_if_requested(cap, cv2.CAP_PROP_BRIGHTNESS, args.brightness, "brightness")
    _set_if_requested(cap, cv2.CAP_PROP_CONTRAST, args.contrast, "contrast")
    _set_if_requested(cap, cv2.CAP_PROP_SATURATION, args.saturation, "saturation")
    _set_if_requested(cap, cv2.CAP_PROP_SHARPNESS, args.sharpness, "sharpness")
    _set_if_requested(
        cap,
        cv2.CAP_PROP_WB_TEMPERATURE,
        args.wb_temperature,
        "wb_temperature",
    )


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


def compute_frame_stats(frame: np.ndarray) -> dict[str, float]:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return {
        "mean": float(np.mean(gray)),
        "min": float(np.min(gray)),
        "max": float(np.max(gray)),
        "p01": float(np.percentile(gray, 1)),
        "p99": float(np.percentile(gray, 99)),
        "sharpness": float(cv2.Laplacian(gray, cv2.CV_64F).var()),
    }


def draw_overlay(frame: np.ndarray, stats: dict[str, float], fps: float) -> np.ndarray:
    output = frame.copy()
    lines = [
        f"FPS: {fps:5.1f}",
        f"Mean gray: {stats['mean']:6.1f}",
        f"Range p01-p99: {stats['p01']:5.1f} - {stats['p99']:5.1f}",
        f"Min/Max: {stats['min']:5.0f} / {stats['max']:5.0f}",
        f"Sharpness: {stats['sharpness']:8.1f}",
        "Washed out if p99 is near 255 for most frames.",
        "Controls: I overlay | P print props | S snapshot | Q quit",
    ]

    cv2.rectangle(output, (10, 10), (430, 175), (0, 0, 0), -1)
    cv2.rectangle(output, (10, 10), (430, 175), (0, 255, 255), 1)
    for idx, line in enumerate(lines):
        cv2.putText(
            output,
            line,
            (20, 35 + idx * 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
    return output


def main() -> int:
    args = parse_args()
    backend = DEFAULT_BACKEND[args.backend]
    cap = cv2.VideoCapture(args.camera, backend)
    if not cap.isOpened():
        print(f"Could not open camera index {args.camera} with backend {args.backend}.")
        return 1

    apply_camera_settings(cap, args)
    print_camera_properties(cap)

    show_overlay = True
    frame_counter = 0
    fps = 0.0
    fps_t0 = time.perf_counter()
    snapshot_dir = Path(__file__).resolve().parent

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Camera frame read failed.")
                return 1

            frame_counter += 1
            now = time.perf_counter()
            elapsed = now - fps_t0
            if elapsed >= 0.5:
                fps = frame_counter / elapsed
                frame_counter = 0
                fps_t0 = now

            stats = compute_frame_stats(frame)
            preview = draw_overlay(frame, stats, fps) if show_overlay else frame
            cv2.imshow("Raw Camera Feed", preview)

            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q"), ord("Q")):
                break
            if key in (ord("i"), ord("I")):
                show_overlay = not show_overlay
            elif key in (ord("p"), ord("P")):
                print_camera_properties(cap)
                print(
                    "Current frame stats: "
                    f"mean={stats['mean']:.1f} p99={stats['p99']:.1f} "
                    f"sharpness={stats['sharpness']:.1f}"
                )
            elif key in (ord("s"), ord("S")):
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                snapshot_path = snapshot_dir / f"camera_snapshot_{timestamp}.png"
                cv2.imwrite(str(snapshot_path), frame)
                print(f"Saved snapshot: {snapshot_path}")
    finally:
        cap.release()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
