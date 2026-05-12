#!/usr/bin/env python3


from __future__ import annotations

import argparse
import contextlib
import io
import math
from pathlib import Path
import signal
import sys
import time

import smbus
from imusensor.MPU9250 import MPU9250


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read two MPU9250 sensors and print absolute and relative orientation."
    )
    parser.add_argument(
        "--bus",
        type=int,
        default=1,
        help="I2C bus number. Raspberry Pi typically uses bus 1.",
    )
    parser.add_argument(
        "--imu1-address",
        type=lambda value: int(value, 0),
        default=0x69,
        help="IMU 1 I2C address. Project default is 0x69.",
    )
    parser.add_argument(
        "--imu2-address",
        type=lambda value: int(value, 0),
        default=0x68,
        help="IMU 2 I2C address. Project default is 0x68.",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=5.0,
        help="Refresh rate in Hz for terminal output.",
    )
    parser.add_argument(
        "--no-calibration",
        action="store_true",
        help="Skip loading calibration files even if they are found.",
    )
    parser.add_argument(
        "--calib1",
        type=Path,
        default=None,
        help="Optional calibration JSON path for IMU 1.",
    )
    parser.add_argument(
        "--calib2",
        type=Path,
        default=None,
        help="Optional calibration JSON path for IMU 2.",
    )
    parser.add_argument(
        "--show-raw",
        action="store_true",
        help="Print raw accel / gyro / mag vectors after each summary line.",
    )
    return parser.parse_args()


def compute_rpy_deg(imu: MPU9250.MPU9250) -> tuple[float, float, float]:
    """Estimate roll, pitch, yaw in degrees from accelerometer and magnetometer."""
    ax, ay, az = imu.AccelVals
    mx, my, mz = imu.MagVals

    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))

    xh = mx * math.cos(pitch) + mz * math.sin(pitch)
    yh = (
        mx * math.sin(roll) * math.sin(pitch)
        + my * math.cos(roll)
        - mz * math.sin(roll) * math.cos(pitch)
    )
    yaw = math.atan2(-yh, xh)

    return tuple(math.degrees(value) for value in (roll, pitch, yaw))


def wrap_deg(angle: float) -> float:
    return (float(angle) + 180.0) % 360.0 - 180.0


def euler_zyx_to_quat(
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
) -> tuple[float, float, float, float]:
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (w, x, y, z)


def quat_conj(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    w, x, y, z = q
    return (w, -x, -y, -z)


def quat_mul(
    q1: tuple[float, float, float, float],
    q2: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


def quat_to_euler_zyx(q: tuple[float, float, float, float]) -> tuple[float, float, float]:
    w, x, y, z = q

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return tuple(wrap_deg(math.degrees(value)) for value in (roll, pitch, yaw))


def compute_relative_rpy(
    imu1_rpy: tuple[float, float, float],
    imu2_rpy: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Return IMU 1 orientation relative to IMU 2."""
    q1 = euler_zyx_to_quat(*imu1_rpy)
    q2 = euler_zyx_to_quat(*imu2_rpy)
    q_rel = quat_mul(quat_conj(q2), q1)
    return quat_to_euler_zyx(q_rel)


def vector_norm(values: tuple[float, float, float] | list[float]) -> float:
    x, y, z = values
    return math.sqrt(x * x + y * y + z * z)


def format_triplet(
    values: tuple[float, float, float] | list[float],
    width: int = 8,
) -> str:
    return " ".join(f"{value:{width}.3f}" for value in values)


def resolve_default_calibration_paths() -> tuple[Path, Path]:
    package_root = Path(__file__).resolve().parents[1]
    return (
        package_root / "share" / "stewart_control" / "calib1.json",
        package_root / "share" / "stewart_control" / "calib2.json",
    )


def probe_device(bus: smbus.SMBus, address: int) -> int | None:
    try:
        return int(bus.read_byte_data(address, 0x75))
    except OSError:
        return None


def begin_sensor(bus: smbus.SMBus, address: int) -> MPU9250.MPU9250:
    imu = MPU9250.MPU9250(bus, address)
    captured_stdout = io.StringIO()
    with contextlib.redirect_stdout(captured_stdout):
        imu.begin()

    startup_note = captured_stdout.getvalue().strip()

    return imu


def load_calibration_if_available(imu: MPU9250.MPU9250, path: Path | None) -> str:
    if path is None:
        return "calibration disabled"
    if not path.is_file():
        return f"calibration missing: {path}"
    imu.loadCalibDataFromFile(str(path))
    return f"calibration loaded: {path}"


def print_header(
    bus: int,
    imu1_address: int,
    imu2_address: int,
    rate: float,
    imu1_calibration_status: str,
    imu2_calibration_status: str,
) -> None:
    print(f"I2C bus   : {bus}")
    print(f"IMU 1     : 0x{imu1_address:02X}  {imu1_calibration_status}")
    print(f"IMU 2     : 0x{imu2_address:02X}  {imu2_calibration_status}")
    print(f"Rate      : {rate:.2f} Hz")
    print(
        " imu1 rpy[deg]".ljust(31)
        + "imu2 rpy[deg]".ljust(31)
        + "relative rpy[deg]".ljust(31)
        + "|a| m/s^2".ljust(12)
        + "|g| rad/s"
    )
    print("-" * 115)


def print_raw_block(label: str, imu: MPU9250.MPU9250) -> None:
    print(
        f"    {label} accel {format_triplet(imu.AccelVals)}  "
        f"gyro {format_triplet(imu.GyroVals)}  "
        f"mag {format_triplet(imu.MagVals)}"
    )


def main() -> int:
    args = parse_args()

    if args.rate <= 0:
        print("Error: --rate must be greater than 0.", file=sys.stderr)
        return 2

    if args.imu1_address == args.imu2_address:
        print("Error: IMU 1 and IMU 2 addresses must be different.", file=sys.stderr)
        return 2

    running = True

    def handle_signal(_signum, _frame) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        bus = smbus.SMBus(args.bus)

        imu1_who = probe_device(bus, args.imu1_address)
        imu2_who = probe_device(bus, args.imu2_address)
        if imu1_who is None:
            raise RuntimeError(f"No device responded at 0x{args.imu1_address:02X}")
        if imu2_who is None:
            raise RuntimeError(f"No device responded at 0x{args.imu2_address:02X}")

        imu1 = begin_sensor(bus, args.imu1_address)
        imu2 = begin_sensor(bus, args.imu2_address)
    except Exception as exc:
        print(
            "Failed to open the IMU pair. Check wiring, I2C enablement, bus number, "
            f"and addresses. Details: {exc}",
            file=sys.stderr,
        )
        return 1

    default_calib1, default_calib2 = resolve_default_calibration_paths()
    calib1_path = None if args.no_calibration else (args.calib1 or default_calib1)
    calib2_path = None if args.no_calibration else (args.calib2 or default_calib2)

    try:
        imu1_calibration_status = load_calibration_if_available(imu1, calib1_path)
        imu2_calibration_status = load_calibration_if_available(imu2, calib2_path)
    except Exception as exc:
        print(f"Failed to load calibration data: {exc}", file=sys.stderr)
        return 1

    print(f"Address 0x{args.imu1_address:02X}: 0x{imu1_who:02X}")
    print(f"Address 0x{args.imu2_address:02X}: 0x{imu2_who:02X}")
    print_header(
        args.bus,
        args.imu1_address,
        args.imu2_address,
        args.rate,
        imu1_calibration_status,
        imu2_calibration_status,
    )

    period_s = 1.0 / args.rate

    while running:
        started = time.monotonic()
        try:
            imu1.readSensor()
            imu2.readSensor()

            imu1_rpy = compute_rpy_deg(imu1)
            imu2_rpy = compute_rpy_deg(imu2)
            relative_rpy = compute_relative_rpy(imu1_rpy, imu2_rpy)
            accel1_norm = vector_norm(imu1.AccelVals)
            accel2_norm = vector_norm(imu2.AccelVals)
            gyro1_norm = vector_norm(imu1.GyroVals)
            gyro2_norm = vector_norm(imu2.GyroVals)

            print(
                f"{imu1_rpy[0]:8.2f} {imu1_rpy[1]:8.2f} {imu1_rpy[2]:8.2f}".ljust(31)
                + f"{imu2_rpy[0]:8.2f} {imu2_rpy[1]:8.2f} {imu2_rpy[2]:8.2f}".ljust(31)
                + f"{relative_rpy[0]:8.2f} {relative_rpy[1]:8.2f} {relative_rpy[2]:8.2f}".ljust(31)
                + f"{accel1_norm:5.2f}/{accel2_norm:5.2f}".ljust(12)
                + f"{gyro1_norm:5.3f}/{gyro2_norm:5.3f}"
            )

            if args.show_raw:
                print_raw_block("IMU1", imu1)
                print_raw_block("IMU2", imu2)
        except KeyboardInterrupt:
            running = False
            continue
        except Exception as exc:
            print(f"Read error: {exc}", file=sys.stderr)

        elapsed = time.monotonic() - started
        time.sleep(max(0.0, period_s - elapsed))

    print("\nStopped.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
