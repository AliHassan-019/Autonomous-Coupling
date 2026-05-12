#!/usr/bin/env python3

from __future__ import annotations

import argparse
import contextlib
import io
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
import smbus
from imusensor.MPU9250 import MPU9250


G_MPS2 = 9.80665


ACCEL_POSES = (
    ("face_1", "Place the board flat with the component side facing up."),
    ("face_2", "Flip the board so the component side faces down."),
    ("face_3", "Stand the board on one long edge."),
    ("face_4", "Stand the board on the opposite long edge."),
    ("face_5", "Stand the board on one short edge."),
    ("face_6", "Stand the board on the opposite short edge."),
)



def parse_args() -> argparse.Namespace:
    package_root = Path(__file__).resolve().parents[1]
    default_calib1 = package_root / "share" / "stewart_control" / "calib1.json"
    default_calib2 = package_root / "share" / "stewart_control" / "calib2.json"

    parser = argparse.ArgumentParser(
        description="Calibrate the Stewart platform's two MPU9250 sensors."
    )
    parser.add_argument("--bus", type=int, default=1, help="I2C bus number.")
    parser.add_argument(
        "--imu1-address",
        type=lambda value: int(value, 0),
        default=0x69,
        help="IMU 1 address. Project default is 0x69.",
    )
    parser.add_argument(
        "--imu2-address",
        type=lambda value: int(value, 0),
        default=0x68,
        help="IMU 2 address. Project default is 0x68.",
    )
    parser.add_argument(
        "--samples-per-face",
        type=int,
        default=250,
        help="Samples to average for each accelerometer face.",
    )
    parser.add_argument(
        "--gyro-samples",
        type=int,
        default=500,
        help="Stationary samples used for gyro bias estimation.",
    )
    parser.add_argument(
        "--mag-samples",
        type=int,
        default=1500,
        help="Samples used for magnetometer calibration.",
    )
    parser.add_argument(
        "--sample-delay",
        type=float,
        default=0.02,
        help="Delay between samples in seconds.",
    )
    parser.add_argument(
        "--settle-seconds",
        type=float,
        default=2.0,
        help="Time to wait after each prompt before sampling begins.",
    )
    parser.add_argument(
        "--calib1-out",
        type=Path,
        default=default_calib1,
        help="Output JSON path for IMU 1.",
    )
    parser.add_argument(
        "--calib2-out",
        type=Path,
        default=default_calib2,
        help="Output JSON path for IMU 2.",
    )
    parser.add_argument(
        "--only",
        choices=("imu1", "imu2", "both"),
        default="both",
        help="Calibrate only one sensor or both sequentially.",
    )
    return parser.parse_args()


def begin_sensor(bus: smbus.SMBus, address: int) -> MPU9250.MPU9250:
    imu = MPU9250.MPU9250(bus, address)
    captured_stdout = io.StringIO()
    with contextlib.redirect_stdout(captured_stdout):
        imu.begin()

    startup_note = captured_stdout.getvalue().strip()
    if startup_note:
        print(f"Library startup note for 0x{address:02X}: {startup_note}", file=sys.stderr)
    return imu


def probe_device(bus: smbus.SMBus, address: int) -> int | None:
    try:
        return int(bus.read_byte_data(address, 0x75))
    except OSError:
        return None


def wait_for_enter(message: str) -> None:
    print()
    input(f"{message}\nPress Enter when ready...")


def countdown(seconds: float) -> None:
    end_time = time.monotonic() + max(0.0, seconds)
    while True:
        remaining = end_time - time.monotonic()
        if remaining <= 0.0:
            print("  starting now...      ")
            return
        print(f"  starting in {remaining:4.1f}s", end="\r", flush=True)
        time.sleep(min(0.1, remaining))


def sample_sensor(
    imu: MPU9250.MPU9250,
    sample_count: int,
    sample_delay: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    accel_samples = np.empty((sample_count, 3), dtype=float)
    gyro_samples = np.empty((sample_count, 3), dtype=float)
    mag_samples = np.empty((sample_count, 3), dtype=float)

    for index in range(sample_count):
        imu.readSensor()
        accel_samples[index] = np.asarray(imu.AccelVals, dtype=float)
        gyro_samples[index] = np.asarray(imu.GyroVals, dtype=float)
        mag_samples[index] = np.asarray(imu.MagVals, dtype=float)
        time.sleep(sample_delay)

    return accel_samples, gyro_samples, mag_samples


def collect_stationary_gyro_bias(
    imu: MPU9250.MPU9250,
    sample_count: int,
    sample_delay: float,
    settle_seconds: float,
) -> np.ndarray:
    wait_for_enter(
        "Place the platform completely still. This step estimates gyro bias."
    )
    countdown(settle_seconds)
    _, gyro_samples, _ = sample_sensor(imu, sample_count, sample_delay)
    return np.mean(gyro_samples, axis=0)


def collect_accel_face_means(
    imu: MPU9250.MPU9250,
    sample_count: int,
    sample_delay: float,
    settle_seconds: float,
) -> dict[str, np.ndarray]:
    face_means: dict[str, np.ndarray] = {}
    print()
    print("Accelerometer six-face calibration")
    print("Present six distinct faces and hold each pose still.")
    for face_name, instructions in ACCEL_POSES:
        wait_for_enter(instructions)
        countdown(settle_seconds)
        accel_samples, _, _ = sample_sensor(imu, sample_count, sample_delay)
        face_means[face_name] = np.mean(accel_samples, axis=0)
        std = np.std(accel_samples, axis=0)
        print(
            f"  captured {face_name}: mean={np.round(face_means[face_name], 4)} "
            f"std={np.round(std, 4)}"
        )
    return face_means


def derive_accel_calibration(
    face_means: dict[str, np.ndarray],
) -> tuple[np.ndarray, np.ndarray, dict[str, float], dict[str, str]]:
    dominant_assignments: dict[tuple[int, int], tuple[str, np.ndarray]] = {}
    pose_mapping: dict[str, str] = {}

    for face_name, mean_vec in face_means.items():
        axis = int(np.argmax(np.abs(mean_vec)))
        sign = 1 if mean_vec[axis] >= 0.0 else -1
        key = (axis, sign)
        axis_name = "XYZ"[axis]
        signed_name = f"{'+' if sign > 0 else '-'}{axis_name}"
        pose_mapping[face_name] = signed_name
        if key in dominant_assignments:
            prev_face, prev_vec = dominant_assignments[key]
            raise RuntimeError(
                "Accelerometer calibration failed: duplicate face coverage detected. "
                f"{face_name} and {prev_face} both map to {signed_name}. "
                "Use six clearly different poses: top, bottom, left, right, front, back."
            )
        dominant_assignments[key] = (face_name, mean_vec)

    required_keys = {
        (0, 1),
        (0, -1),
        (1, 1),
        (1, -1),
        (2, 1),
        (2, -1),
    }
    if set(dominant_assignments) != required_keys:
        missing = required_keys.difference(dominant_assignments)
        missing_labels = [f"{'+' if sign > 0 else '-'}{'XYZ'[axis]}" for axis, sign in sorted(missing)]
        raise RuntimeError(
            "Accelerometer calibration failed: incomplete six-face coverage. "
            f"Missing dominant orientations: {', '.join(missing_labels)}."
        )

    plus_x = dominant_assignments[(0, 1)][1][0]
    minus_x = dominant_assignments[(0, -1)][1][0]
    plus_y = dominant_assignments[(1, 1)][1][1]
    minus_y = dominant_assignments[(1, -1)][1][1]
    plus_z = dominant_assignments[(2, 1)][1][2]
    minus_z = dominant_assignments[(2, -1)][1][2]

    bias = np.array(
        [
            (plus_x + minus_x) / 2.0,
            (plus_y + minus_y) / 2.0,
            (plus_z + minus_z) / 2.0,
        ],
        dtype=float,
    )

    half_ranges = np.array(
        [
            (plus_x - minus_x) / 2.0,
            (plus_y - minus_y) / 2.0,
            (plus_z - minus_z) / 2.0,
        ],
        dtype=float,
    )

    if np.any(np.abs(half_ranges) < 1e-3):
        raise RuntimeError("Accelerometer calibration failed: one axis half-range is too small.")

    scales = np.abs(G_MPS2 / half_ranges)

    residuals: dict[str, float] = {}
    for face_name, mean_vec in face_means.items():
        signed_axis = pose_mapping[face_name]
        expected = np.zeros(3, dtype=float)
        axis = "XYZ".index(signed_axis[1])
        expected[axis] = G_MPS2 if signed_axis[0] == "+" else -G_MPS2
        corrected = (mean_vec - bias) * scales
        residuals[face_name] = float(np.linalg.norm(corrected - expected))

    if np.any(scales < 0.5) or np.any(scales > 1.5):
        raise RuntimeError(
            "Accelerometer calibration failed: derived scale factors are unrealistic. "
            f"Computed scales: {np.round(scales, 6)}"
        )

    return scales, bias, residuals, pose_mapping


def collect_mag_samples(
    imu: MPU9250.MPU9250,
    sample_count: int,
    sample_delay: float,
    settle_seconds: float,
) -> np.ndarray:
    wait_for_enter(
        "Magnetometer calibration: rotate the sensor slowly through as many 3D "
        "orientations as possible during the full capture."
    )
    countdown(settle_seconds)
    mag_samples = np.empty((sample_count, 3), dtype=float)
    progress_every = max(1, sample_count // 20)
    for index in range(sample_count):
        imu.readSensor()
        mag_samples[index] = np.asarray(imu.MagVals, dtype=float)
        if index % progress_every == 0 or index == sample_count - 1:
            progress = 100.0 * (index + 1) / sample_count
            print(f"  magnetometer capture: {progress:5.1f}%", end="\r", flush=True)
        time.sleep(sample_delay)
    print("  magnetometer capture: 100.0%")
    return mag_samples


def derive_mag_calibration(mag_samples: np.ndarray) -> tuple[np.ndarray, np.ndarray, dict[str, float]]:
    mag_bias = 0.5 * (np.max(mag_samples, axis=0) + np.min(mag_samples, axis=0))
    centered = mag_samples - mag_bias

    covariance = np.cov(centered, rowvar=False)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    eigenvalues = np.clip(eigenvalues, 1e-12, None)

    whitening = eigenvectors @ np.diag(1.0 / np.sqrt(eigenvalues)) @ eigenvectors.T
    whitened = centered @ whitening.T
    radii = np.linalg.norm(whitened, axis=1)
    mean_radius = float(np.mean(radii))
    if mean_radius <= 1e-12:
        raise RuntimeError("Magnetometer calibration failed: insufficient field variation.")

    mag_transform = whitening / mean_radius
    corrected = centered @ mag_transform.T
    corrected_radii = np.linalg.norm(corrected, axis=1)

    metrics = {
        "raw_radius_std": float(np.std(np.linalg.norm(centered, axis=1))),
        "corrected_radius_mean": float(np.mean(corrected_radii)),
        "corrected_radius_std": float(np.std(corrected_radii)),
    }
    return mag_bias, mag_transform, metrics


def build_calibration_payload(
    accel_scales: np.ndarray,
    accel_bias: np.ndarray,
    gyro_bias: np.ndarray,
    mag_bias: np.ndarray,
    mag_transform: np.ndarray,
) -> dict[str, object]:
    return {
        "Accels": accel_scales.tolist(),
        "AccelBias": accel_bias.tolist(),
        "GyroBias": gyro_bias.tolist(),
        "Mags": [1.0, 1.0, 1.0],
        "MagBias": mag_bias.tolist(),
        "Magtransform": mag_transform.tolist(),
    }


def save_json(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2)
        handle.write("\n")


def calibrate_one_sensor(
    bus: smbus.SMBus,
    label: str,
    address: int,
    output_path: Path,
    args: argparse.Namespace,
) -> None:
    print()
    print("=" * 72)
    print(f"{label} calibration at address 0x{address:02X}")
    print("=" * 72)

    who_am_i = probe_device(bus, address)
    if who_am_i is None:
        raise RuntimeError(f"No device responded at 0x{address:02X}")
    print(f"WHO_AM_I: 0x{who_am_i:02X}")

    imu = begin_sensor(bus, address)

    gyro_bias = collect_stationary_gyro_bias(
        imu,
        sample_count=args.gyro_samples,
        sample_delay=args.sample_delay,
        settle_seconds=args.settle_seconds,
    )
    print(f"Gyro bias estimate: {np.round(gyro_bias, 6)}")

    face_means = collect_accel_face_means(
        imu,
        sample_count=args.samples_per_face,
        sample_delay=args.sample_delay,
        settle_seconds=args.settle_seconds,
    )
    accel_scales, accel_bias, accel_residuals, pose_mapping = derive_accel_calibration(face_means)
    print("Detected face mapping:")
    for face_name, signed_axis in pose_mapping.items():
        print(f"  {face_name}: {signed_axis}")
    print(f"Accel scales: {np.round(accel_scales, 6)}")
    print(f"Accel bias  : {np.round(accel_bias, 6)}")
    print("Accel residuals by face (m/s^2):")
    for face_name, residual in accel_residuals.items():
        print(f"  {face_name:<5} ({pose_mapping[face_name]:>3}) {residual:.4f}")

    mag_samples = collect_mag_samples(
        imu,
        sample_count=args.mag_samples,
        sample_delay=args.sample_delay,
        settle_seconds=args.settle_seconds,
    )
    mag_bias, mag_transform, mag_metrics = derive_mag_calibration(mag_samples)
    print(f"Mag bias: {np.round(mag_bias, 6)}")
    print("Mag transform:")
    print(np.round(mag_transform, 6))
    print("Mag quality summary:")
    for key, value in mag_metrics.items():
        print(f"  {key}: {value:.6f}")

    payload = build_calibration_payload(
        accel_scales=accel_scales,
        accel_bias=accel_bias,
        gyro_bias=gyro_bias,
        mag_bias=mag_bias,
        mag_transform=mag_transform,
    )
    save_json(output_path, payload)
    print(f"Saved calibration to {output_path}")


def main() -> int:
    args = parse_args()

    if args.samples_per_face <= 0 or args.gyro_samples <= 0 or args.mag_samples <= 0:
        print("Error: sample counts must be positive.", file=sys.stderr)
        return 2
    if args.sample_delay <= 0.0:
        print("Error: --sample-delay must be greater than 0.", file=sys.stderr)
        return 2
    if args.imu1_address == args.imu2_address and args.only == "both":
        print("Error: IMU addresses must differ when calibrating both sensors.", file=sys.stderr)
        return 2

    try:
        bus = smbus.SMBus(args.bus)
        if args.only in ("imu1", "both"):
            calibrate_one_sensor(bus, "IMU 1", args.imu1_address, args.calib1_out, args)
        if args.only in ("imu2", "both"):
            calibrate_one_sensor(bus, "IMU 2", args.imu2_address, args.calib2_out, args)
    except KeyboardInterrupt:
        print("\nCalibration interrupted.")
        return 130
    except Exception as exc:
        print(f"Calibration failed: {exc}", file=sys.stderr)
        return 1

    print()
    print("Calibration complete.")
    print("Recommended next step:")
    print("  Run test_imu_sensor.py first with --show-raw, then without --no-calibration.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
