
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import math
import argparse
import sys
from pathlib import Path

import smbus
from imusensor.MPU9250 import MPU9250

def wrap_deg(angle_deg: float) -> float:
    return (angle_deg + 180.0) % 360.0 - 180.0

def euler_zyx_to_quat(roll_deg: float, pitch_deg: float, yaw_deg: float):
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)

    cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
    cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
    cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y_ = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (w, x, y_, z)

def quat_conj(q):
    w, x, y, z = q
    return (w, -x, -y, -z)

def quat_mul(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    )

def quat_to_euler_zyx(q):
    w, x, y, z = q

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))

def ensure_i2c_bus(bus_id: int):
    dev = Path(f"/dev/i2c-{bus_id}")
    if not dev.exists():
        raise FileNotFoundError(f"{dev} n'existe pas. (ls /dev/i2c-*)")
    return smbus.SMBus(bus_id)

def load_calib_or_fail(imu, calib_path: Path, label: str):
    if not calib_path.exists():
        raise FileNotFoundError(f"Calib {label} introuvable: {calib_path}")
    imu.loadCalibDataFromFile(str(
calib_path))


def main():
    parser = argparse.ArgumentParser(
description="2 MPU9250 + orientation relative (quaternions).")


    parser.add_argument("--bus", type=int, default=1)
    parser.add_argument("--addr1", type=lambda x: int(x, 0), default=0x69)
    parser.add_argument("--addr2", type=lambda x: int(x, 0), default=0x68)
    parser.add_argument("--calib1"
, type=str, default="/home/rem/mpu9250/imu_19/calib_6faces_mag00.json")
    parser.add_argument("--calib2", type=str, default="/home/rem/mpu9250/imu_19/calib_6faces_mag01.json")

    parser.add_argument("--dt", type=float, default=0.1)

    parser.add_argument("--
relative",

                        choices=["imu1_wrt_imu2", "imu2_wrt_imu1"],
                        default="imu1_wrt_imu2")

    parser.add_argument("--no_mag_
yaw", action="store_true",

                        help="Force yaw=0 (test anti-perturbations mag).")

    parser.add_argument("--print_
mode",

                        choices=["block", "singleline"],
                        default="block",
                        help="block = 3 lignes + séparateur ; singleline = 1 ligne stable.")

    args = parser.parse_args()

    try:
        sys.stdout.reconfigure(line_
buffering=True)

    except Exception:
        pass

    bus = ensure_i2c_bus(args.bus)

    imu1 = MPU9250.MPU9250(bus, args.addr1)
    imu2 = MPU9250.MPU9250(bus, args.addr2)
    imu1.begin(); imu2.begin()

    load_calib_or_fail(imu1, Path(args.calib1), "IMU1")
    load_calib_or_fail(imu2, Path(args.calib2), "IMU2")

    print("IMU1 et IMU2 initialisées et calibrées ✅", flush=True)
    print(f"Bus=/dev/i2c-{args.
bus} addr1={hex(args.addr1)} addr2={hex(args.addr2)}", flush=True)
    print(f"Relatif={args.relative} no_mag_yaw={args.no_mag_yaw} print_mode={args.print_mode}\n", flush=True)


    while True:
        imu1.readSensor(); imu1.computeOrientation()
        imu2.readSensor(); imu2.computeOrientation()

        r1 = wrap_deg(float(imu1.roll));  p1 = wrap_deg(float(imu1.pitch)); y1 = wrap_deg(float(imu1.yaw))
        r2 = wrap_deg(float(imu2.roll));  p2 = wrap_deg(float(imu2.pitch)); y2 = wrap_deg(float(imu2.yaw))

        if args.no_mag_yaw:
            y1 = 0.0; y2 = 0.0

        q1 = euler_zyx_to_quat(r1, p1, y1)
        q2 = euler_zyx_to_quat(r2, p2, y2)

        if args.relative == "imu1_wrt_imu2":
            q_rel = quat_mul(quat_conj(q2), q1)
        else:
            q_rel = quat_mul(quat_conj(q1), q2)

        rr, pr, yr = quat_to_euler_zyx(q_rel)
        rr, pr, yr = wrap_deg(rr), wrap_deg(pr), wrap_deg(yr)

        if args.print_mode == "block":
            print(f"IMU1 → Roll: {r1:7.2f}° | Pitch: {p1:7.2f}° | Yaw: {y1:7.2f}°", flush=True)
            print(f"IMU2 → Roll: {r2:7.2f}° | Pitch: {p2:7.2f}° | Yaw: {y2:7.2f}°", flush=True)
            print(f"Relatif → Roll: {rr:7.2f}° | Pitch: {pr:7.2f}° | Yaw: {yr:7.2f}°", flush=True)
            print("-" * 60, flush=True)
        else:
            line = (f"IMU1(r={r1:6.2f},p={p1:6.
2f},y={y1:7.2f})  "

                    f"IMU2(r={r2:6.2f},p={p2:6.2f}
,y={y2:7.2f})  "

                    f"REL(r={rr:6.2f},p={pr:6.2f},
y={yr:7.2f})")

            print(line, flush=True)

        time.sleep(max(0.01, args.dt))

if __name__ == "__main__":
    main()
