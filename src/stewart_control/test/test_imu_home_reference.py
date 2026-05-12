#!/usr/bin/env python3

from __future__ import annotations

import argparse
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
from stewart_control.imu_node import (  # noqa: E402
    compute_rpy,
    resolve_calibration_path,
    wrap_rpy_deg,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Measure the current IMU orientation at the platform home position "
            "and print a YAML-ready reference_orientation_deg value."
        )
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=150,
        help="Number of IMU samples to average. Default: 150.",
    )
    parser.add_argument(
        "--sample-delay",
        type=float,
        default=0.02,
        help="Delay between samples in seconds. Default: 0.02.",
    )
    parser.add_argument(
        "--show-progress",
        action="store_true",
        help="Print each sampled orientation while collecting data.",
    )
    return parser.parse_args()


def circular_mean_deg(samples: np.ndarray) -> np.ndarray:
    radians = np.radians(samples)
    mean_sin = np.mean(np.sin(radians), axis=0)
    mean_cos = np.mean(np.cos(radians), axis=0)
    return wrap_rpy_deg(np.degrees(np.arctan2(mean_sin, mean_cos)))


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
        calibration_path = resolve_calibration_path(
            imu_cfg.get("calibration_path"),
            "calib2.json",
        )
        imu.loadCalibDataFromFile(calibration_path)
    except Exception as exc:
        print(
            "Failed to initialize the configured IMU. "
            f"Details: {exc}",
            file=sys.stderr,
        )
        return 1

    print("Keep the platform fixed at the desired home position.")
    print(f"IMU address      : 0x{int(imu_cfg['imu_address']):02X}")
    print(f"Calibration file : {calibration_path}")
    print(f"Samples          : {args.samples}")
    print(f"Sample delay     : {args.sample_delay:.3f} s")
    print("")

    samples: list[np.ndarray] = []

    for index in range(args.samples):
        if not running:
            break
        try:
            imu.readSensor()
            sample = wrap_rpy_deg(np.array(compute_rpy(imu), dtype=float))
            samples.append(sample)
            if args.show_progress:
                print(
                    f"[{index + 1:03d}/{args.samples:03d}] "
                    f"R={sample[0]:8.2f}  P={sample[1]:8.2f}  Y={sample[2]:8.2f}"
                )
            time.sleep(args.sample_delay)
        except Exception as exc:
            print(f"Read error: {exc}", file=sys.stderr)
            return 1

    if not samples:
        print("No IMU samples were collected.", file=sys.stderr)
        return 1

    sample_array = np.vstack(samples)
    mean_rpy = circular_mean_deg(sample_array)
    std_rpy = np.std(sample_array, axis=0)
    current_reference = wrap_rpy_deg(
        np.array(imu_cfg.get("reference_orientation_deg", [0.0, 0.0, 0.0]), dtype=float)
    )

    print("Suggested home orientation reference (degrees):")
    print(
        f"  roll={mean_rpy[0]:.2f}, pitch={mean_rpy[1]:.2f}, yaw={mean_rpy[2]:.2f}"
    )
    print("Sample stability (standard deviation, degrees):")
    print(
        f"  roll={std_rpy[0]:.2f}, pitch={std_rpy[1]:.2f}, yaw={std_rpy[2]:.2f}"
    )
    print("Current YAML reference_orientation_deg:")
    print(
        f"  [{current_reference[0]:.2f}, {current_reference[1]:.2f}, {current_reference[2]:.2f}]"
    )
    print("Paste this into config/stewart_params.yaml:")
    print(
        f"  reference_orientation_deg: "
        f"[{mean_rpy[0]:.2f}, {mean_rpy[1]:.2f}, {mean_rpy[2]:.2f}]"
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
