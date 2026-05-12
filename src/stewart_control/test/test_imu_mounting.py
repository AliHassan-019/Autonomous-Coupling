#!/usr/bin/env python3

from __future__ import annotations

import argparse
import itertools
import signal
import sys
import time
from pathlib import Path

import numpy as np
import smbus
from imusensor.MPU9250 import MPU9250


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from stewart_control.config_loader import get_config  # noqa: E402
from stewart_control.imu_node import compute_rpy, resolve_calibration_path, wrap_rpy_deg  # noqa: E402


PLATFORM_MOTIONS = (
    (
        "roll",
        "Tilt the moving platform to positive ROLL and hold it still.",
    ),
    (
        "pitch",
        "Tilt the moving platform to positive PITCH and hold it still.",
    ),
    (
        "yaw",
        "Rotate the moving platform to positive YAW and hold it still.",
    ),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Guide an interactive IMU mounting check and suggest the orientation axis "
            "remap for imu_node."
        )
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=120,
        help="Number of samples to average for each pose. Default: 120.",
    )
    parser.add_argument(
        "--sample-delay",
        type=float,
        default=0.02,
        help="Delay between samples in seconds. Default: 0.02.",
    )
    parser.add_argument(
        "--min-delta-deg",
        type=float,
        default=5.0,
        help="Minimum dominant axis change required for each motion. Default: 5 deg.",
    )
    parser.add_argument(
        "--skip-calibration",
        action="store_true",
        help="Do not load the configured calibration JSON before sampling.",
    )
    return parser.parse_args()


def circular_mean_deg(samples: np.ndarray) -> np.ndarray:
    radians = np.radians(samples)
    mean_sin = np.mean(np.sin(radians), axis=0)
    mean_cos = np.mean(np.cos(radians), axis=0)
    return wrap_rpy_deg(np.degrees(np.arctan2(mean_sin, mean_cos)))


def sample_mean_rpy(
    imu: MPU9250.MPU9250,
    sample_count: int,
    sample_delay: float,
) -> np.ndarray:
    samples = np.empty((sample_count, 3), dtype=float)
    for index in range(sample_count):
        imu.readSensor()
        samples[index] = wrap_rpy_deg(np.array(compute_rpy(imu), dtype=float))
        time.sleep(sample_delay)
    return circular_mean_deg(samples)


def prompt_and_capture(
    imu: MPU9250.MPU9250,
    label: str,
    instructions: str,
    sample_count: int,
    sample_delay: float,
) -> np.ndarray:
    print()
    input(f"{instructions}\nPress Enter when ready to capture {label}...")
    print("Capturing...")
    mean_rpy = sample_mean_rpy(imu, sample_count, sample_delay)
    print(
        f"{label:>8s} mean -> "
        f"R={mean_rpy[0]:+7.2f}  P={mean_rpy[1]:+7.2f}  Y={mean_rpy[2]:+7.2f}"
    )
    return mean_rpy


def infer_axis_mapping(
    baseline: np.ndarray,
    motion_measurements: dict[str, np.ndarray],
    min_delta_deg: float,
) -> tuple[list[int], list[int], dict[str, dict[str, float | int | str]]]:
    platform_axes = ("roll", "pitch", "yaw")
    deltas_by_axis: dict[str, np.ndarray] = {}
    report: dict[str, dict[str, float | int | str]] = {}

    for platform_axis in platform_axes:
        delta = wrap_rpy_deg(motion_measurements[platform_axis] - baseline)
        deltas_by_axis[platform_axis] = delta
        dominant_axis = int(np.argmax(np.abs(delta)))
        dominant_value = float(delta[dominant_axis])
        if abs(dominant_value) < min_delta_deg:
            raise RuntimeError(
                f"{platform_axis} motion was too small to infer mounting. "
                f"Dominant change was only {dominant_value:.2f} deg."
            )

    best_perm: tuple[int, int, int] | None = None
    best_score = float("-inf")
    for perm in itertools.permutations((0, 1, 2)):
        score = sum(
            abs(float(deltas_by_axis[platform_axis][sensor_axis]))
            for platform_axis, sensor_axis in zip(platform_axes, perm)
        )
        if score > best_score:
            best_score = score
            best_perm = perm

    if best_perm is None:
        raise RuntimeError("Unable to infer any valid IMU axis mapping.")

    inferred_order = list(best_perm)
    inferred_signs: list[int] = []

    for platform_axis, sensor_axis in zip(platform_axes, inferred_order):
        delta = deltas_by_axis[platform_axis]
        chosen_value = float(delta[sensor_axis])
        ranked_axes = np.argsort(np.abs(delta))[::-1]
        top_axis = int(ranked_axes[0])
        second_axis = int(ranked_axes[1])
        top_value = float(delta[top_axis])
        second_value = float(delta[second_axis])
        margin = abs(top_value) - abs(second_value)

        inferred_signs.append(1 if chosen_value >= 0.0 else -1)
        report[platform_axis] = {
            "delta_roll_deg": float(delta[0]),
            "delta_pitch_deg": float(delta[1]),
            "delta_yaw_deg": float(delta[2]),
            "dominant_sensor_axis": ("roll", "pitch", "yaw")[top_axis],
            "dominant_sensor_index": top_axis,
            "dominant_delta_deg": top_value,
            "selected_sensor_axis": ("roll", "pitch", "yaw")[sensor_axis],
            "selected_sensor_index": sensor_axis,
            "selected_delta_deg": chosen_value,
            "second_sensor_axis": ("roll", "pitch", "yaw")[second_axis],
            "second_delta_deg": second_value,
            "selection_margin_deg": float(margin),
            "selection_note": (
                "clear"
                if sensor_axis == top_axis and margin >= min_delta_deg
                else "ambiguous"
            ),
        }

    return inferred_order, inferred_signs, report


def main() -> int:
    args = parse_args()
    if args.samples <= 0:
        print("Error: --samples must be greater than 0.", file=sys.stderr)
        return 2
    if args.sample_delay < 0.0:
        print("Error: --sample-delay must be non-negative.", file=sys.stderr)
        return 2

    cfg = get_config()
    imu_cfg = cfg["imu"]

    running = True

    def handle_signal(_signum, _frame) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        bus = smbus.SMBus(imu_cfg["i2c_bus"])
        imu = MPU9250.MPU9250(bus, imu_cfg["imu_address"])
        imu.begin()
        calibration_path = None
        if not args.skip_calibration:
            calibration_path = resolve_calibration_path(
                imu_cfg.get("calibration_path"),
                "calib2.json",
            )
            imu.loadCalibDataFromFile(calibration_path)
    except Exception as exc:
        print(f"Failed to initialize the configured IMU: {exc}", file=sys.stderr)
        return 1

    print("IMU mounting check")
    print(f"  IMU address  : 0x{int(imu_cfg['imu_address']):02X}")
    print(f"  Samples/pose : {args.samples}")
    print(f"  Sample delay : {args.sample_delay:.3f} s")
    if calibration_path is not None:
        print(f"  Calibration  : {calibration_path}")
    else:
        print("  Calibration  : skipped")

    try:
        baseline = prompt_and_capture(
            imu,
            "baseline",
            "Keep the moving platform in its neutral home position and keep it still.",
            args.samples,
            args.sample_delay,
        )
        if not running:
            print("Interrupted.")
            return 130

        motion_measurements: dict[str, np.ndarray] = {}
        for axis_name, instructions in PLATFORM_MOTIONS:
            motion_measurements[axis_name] = prompt_and_capture(
                imu,
                axis_name,
                instructions,
                args.samples,
                args.sample_delay,
            )
            if not running:
                print("Interrupted.")
                return 130

        axis_order, axis_sign, report = infer_axis_mapping(
            baseline,
            motion_measurements,
            args.min_delta_deg,
        )
    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 130
    except Exception as exc:
        print(f"Mounting check failed: {exc}", file=sys.stderr)
        return 1

    print()
    print("Detected platform-to-IMU mapping")
    for platform_axis in ("roll", "pitch", "yaw"):
        details = report[platform_axis]
        print(
            f"  {platform_axis:<5} -> sensor {details['selected_sensor_axis']} "
            f"(index {details['selected_sensor_index']}, "
            f"delta {details['selected_delta_deg']:+.2f} deg, "
            f"{details['selection_note']})"
        )
        print(
            f"           raw deltas: "
            f"R={details['delta_roll_deg']:+.2f}  "
            f"P={details['delta_pitch_deg']:+.2f}  "
            f"Y={details['delta_yaw_deg']:+.2f}"
        )
        print(
            f"           strongest axis was {details['dominant_sensor_axis']} "
            f"({details['dominant_delta_deg']:+.2f} deg), "
            f"runner-up {details['second_sensor_axis']} "
            f"({details['second_delta_deg']:+.2f} deg), "
            f"margin {details['selection_margin_deg']:+.2f} deg"
        )

    print()
    print("Paste this into config/stewart_params.yaml under imu:")
    print(f"  orientation_axis_order: {axis_order}")
    print(f"  orientation_axis_sign: {axis_sign}")
    print()
    print("Then re-run test_imu_home_reference.py to refresh reference_orientation_deg.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
