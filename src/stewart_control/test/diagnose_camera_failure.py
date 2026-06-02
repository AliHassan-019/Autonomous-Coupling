#!/usr/bin/env python3
"""Diagnose USB/V4L2/OpenCV camera stalls outside ROS.

The script is intentionally standalone so it can be run while ROS is stopped.
It checks device ownership, V4L2 state, frame-read latency, optional exposure
profiles, and recent kernel messages, then prints the most likely failure cause.
"""

from __future__ import annotations

import argparse
import shutil
import statistics
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path

import cv2


BACKENDS = {
    "any": cv2.CAP_ANY,
    "v4l2": getattr(cv2, "CAP_V4L2", cv2.CAP_ANY),
}

FOURCC = {
    "YUY2": cv2.VideoWriter_fourcc(*"YUY2"),
    "MJPG": cv2.VideoWriter_fourcc(*"MJPG"),
}


@dataclass
class ReadStats:
    label: str
    opened: bool
    frames: int
    failures: int
    slow_reads: int
    max_s: float
    mean_s: float
    p95_s: float
    fps: float


def run_cmd(args: list[str], timeout_s: float = 4.0) -> str:
    if shutil.which(args[0]) is None:
        return f"{args[0]} not installed"

    try:
        result = subprocess.run(
            args,
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout_s,
        )
    except subprocess.TimeoutExpired:
        return f"{' '.join(args)} timed out after {timeout_s:.1f}s"
    except OSError as exc:
        return f"{' '.join(args)} failed: {exc}"

    output = (result.stdout + result.stderr).strip()
    return output if output else "(no output)"


def set_if_supported(cap: cv2.VideoCapture, prop_id: int, value: float, label: str) -> None:
    ok = cap.set(prop_id, value)
    actual = cap.get(prop_id)
    print(f"  {label:18s} request={value!r:<8} applied={ok!s:<5} readback={actual}")


def apply_common_settings(cap: cv2.VideoCapture, args: argparse.Namespace) -> None:
    set_if_supported(cap, cv2.CAP_PROP_FRAME_WIDTH, float(args.width), "width")
    set_if_supported(cap, cv2.CAP_PROP_FRAME_HEIGHT, float(args.height), "height")
    set_if_supported(cap, cv2.CAP_PROP_BUFFERSIZE, 1.0, "buffersize")
    if args.fourcc:
        set_if_supported(cap, cv2.CAP_PROP_FOURCC, float(FOURCC[args.fourcc]), "fourcc")
    if hasattr(cv2, "CAP_PROP_OPEN_TIMEOUT_MSEC"):
        cap.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, float(args.open_timeout_ms))
    if hasattr(cv2, "CAP_PROP_READ_TIMEOUT_MSEC"):
        cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, float(args.read_timeout_ms))


def apply_profile(cap: cv2.VideoCapture, profile: str, exposure: float | None) -> None:
    if profile == "current":
        return

    if profile == "auto_exposure":
        set_if_supported(cap, cv2.CAP_PROP_AUTO_EXPOSURE, 0.75, "auto_exposure")
        return

    if profile == "manual_exposure":
        set_if_supported(cap, cv2.CAP_PROP_AUTO_EXPOSURE, 0.25, "auto_exposure")
        if exposure is not None:
            set_if_supported(cap, cv2.CAP_PROP_EXPOSURE, float(exposure), "exposure")
        return

    raise ValueError(f"Unknown profile: {profile}")


def sample_reads(
    label: str,
    args: argparse.Namespace,
    profile: str,
    duration_s: float,
) -> ReadStats:
    print()
    print(f"=== OpenCV read test: {label} ===")
    cap = cv2.VideoCapture(args.camera, BACKENDS[args.backend])
    if not cap.isOpened():
        print("  Result: could not open camera")
        return ReadStats(label, False, 0, 0, 0, 0.0, 0.0, 0.0, 0.0)

    try:
        apply_common_settings(cap, args)
        apply_profile(cap, profile, args.exposure)
        time.sleep(args.settle_s)

        durations: list[float] = []
        failures = 0
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            started = time.monotonic()
            ok, _frame = cap.read()
            elapsed = time.monotonic() - started
            if not ok:
                failures += 1
                continue
            durations.append(elapsed)

        frames = len(durations)
        max_s = max(durations, default=0.0)
        mean_s = statistics.fmean(durations) if durations else 0.0
        p95_s = percentile(durations, 95.0)
        slow_reads = sum(1 for value in durations if value >= args.slow_read_s)
        fps = frames / duration_s if duration_s > 0 else 0.0
        print(
            "  frames={frames} failures={failures} fps={fps:.1f} "
            "mean={mean_s:.3f}s p95={p95_s:.3f}s max={max_s:.3f}s "
            "slow_reads={slow_reads}".format(
                frames=frames,
                failures=failures,
                fps=fps,
                mean_s=mean_s,
                p95_s=p95_s,
                max_s=max_s,
                slow_reads=slow_reads,
            )
        )
        return ReadStats(label, True, frames, failures, slow_reads, max_s, mean_s, p95_s, fps)
    finally:
        cap.release()


def percentile(values: list[float], pct: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    idx = int(round((len(ordered) - 1) * pct / 100.0))
    return ordered[idx]


def print_system_checks(device: str) -> tuple[str, str, str]:
    print("=== Device ownership ===")
    owner_output = run_cmd(["fuser", "-v", device])
    print(owner_output)

    print()
    print("=== V4L2 controls ===")
    controls_output = run_cmd(["v4l2-ctl", "-d", device, "--all"], timeout_s=5.0)
    print(controls_output)

    print()
    print("=== Recent kernel USB/video messages ===")
    dmesg_output = run_cmd(["dmesg", "--ctime", "--level=err,warn"], timeout_s=5.0)
    filtered = filter_relevant_kernel_lines(dmesg_output)
    print(filtered)

    return owner_output, controls_output, filtered


def filter_relevant_kernel_lines(text: str) -> str:
    if not text or "not installed" in text or "Operation not permitted" in text:
        return text
    keywords = ("usb", "uvc", "video", "camera", "v4l", "reset", "disconnect", "over-current")
    lines = [line for line in text.splitlines() if any(key in line.lower() for key in keywords)]
    return "\n".join(lines[-80:]) if lines else "(no recent USB/video warnings found)"


def infer_reason(
    stats: list[ReadStats],
    owner_output: str,
    controls_output: str,
    kernel_output: str,
    args: argparse.Namespace,
) -> list[str]:
    reasons: list[str] = []
    running_process_conflict = (
        device_has_process(owner_output)
        and "diagnose_camera_failure.py" not in owner_output
    )
    if running_process_conflict:
        reasons.append(
            "Another process appears to be using the camera. Stop ROS camera nodes, "
            "web browsers, Cheese, or other preview tools and test again."
        )

    if any(not item.opened for item in stats):
        reasons.append(
            "The camera could not be opened in at least one test. This points to a "
            "busy device, driver lockup, disconnected camera, or wrong /dev/video index."
        )

    worst = max((item.max_s for item in stats if item.opened), default=0.0)
    total_failures = sum(item.failures for item in stats)
    total_slow = sum(item.slow_reads for item in stats)
    if worst >= args.very_slow_read_s or total_failures > 0:
        reasons.append(
            "OpenCV read latency/failures are high. The stall is happening below ROS, "
            "inside the camera/USB/V4L2 capture path."
        )
    elif total_slow > 0:
        reasons.append(
            "Some reads are slower than expected. This can create visible lag even if "
            "the camera does not fully disconnect."
        )

    if has_kernel_usb_problem(kernel_output):
        reasons.append(
            "Kernel logs contain USB/video warnings. This strongly suggests USB cable, "
            "port, hub, power, or UVC driver instability."
        )

    if dynamic_exposure_framerate_enabled(controls_output):
        reasons.append(
            "V4L2 reports auto exposure/aperture priority with dynamic framerate enabled. "
            "That allows the camera to lower FPS and can explain slow, uneven cap.read() "
            "times. Disable dynamic framerate and use a fixed exposure time."
        )

    if "exposure" in controls_output.lower():
        manual = next((item for item in stats if item.label == "manual_exposure"), None)
        auto = next((item for item in stats if item.label == "auto_exposure"), None)
        if manual and auto and manual.opened and auto.opened:
            if manual.max_s < auto.max_s * 0.6 and auto.max_s >= args.slow_read_s:
                reasons.append(
                    "Auto exposure is much slower than manual exposure. Disable auto "
                    "exposure and use a fixed exposure/gain."
                )
            elif auto.max_s < manual.max_s * 0.6 and manual.max_s >= args.slow_read_s:
                reasons.append(
                    "The selected manual exposure profile is slower or unstable. Try a "
                    "shorter exposure time or let the driver use auto exposure."
                )

    if not reasons:
        reasons.append(
            "No clear camera fault was reproduced during this run. Increase --duration "
            "or run while the same USB/camera setup is connected as in ROS."
        )

    return reasons


def device_has_process(output: str) -> bool:
    ignored = ("not installed", "no output", "specified filename")
    lowered = output.lower()
    return bool(output.strip()) and not any(item in lowered for item in ignored)


def has_kernel_usb_problem(output: str) -> bool:
    lowered = output.lower()
    if "operation not permitted" in lowered or "not installed" in lowered:
        return False
    indicators = (
        "reset",
        "disconnect",
        "unable to",
        "failed",
        "error",
        "timeout",
        "over-current",
        "device descriptor read",
        "not responding",
    )
    return any(indicator in lowered for indicator in indicators)


def dynamic_exposure_framerate_enabled(output: str) -> bool:
    lowered = output.lower()
    return (
        "auto_exposure" in lowered
        and "aperture priority mode" in lowered
        and "exposure_dynamic_framerate" in lowered
        and "value=1" in lowered
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Diagnose camera stalls from OpenCV, V4L2, and system logs."
    )
    parser.add_argument("--camera", type=int, default=0, help="OpenCV camera index.")
    parser.add_argument("--device", default="/dev/video0", help="Linux video device.")
    parser.add_argument("--backend", choices=sorted(BACKENDS), default="v4l2")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fourcc", choices=sorted(FOURCC), default=None)
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--settle-s", type=float, default=0.5)
    parser.add_argument("--exposure", type=float, default=-7.0)
    parser.add_argument("--slow-read-s", type=float, default=0.15)
    parser.add_argument("--very-slow-read-s", type=float, default=0.5)
    parser.add_argument("--open-timeout-ms", type=int, default=1000)
    parser.add_argument("--read-timeout-ms", type=int, default=1000)
    parser.add_argument(
        "--profiles",
        choices=("current", "manual", "auto", "all"),
        default="all",
        help="Exposure profiles to test.",
    )
    return parser.parse_args()


def selected_profiles(name: str) -> list[tuple[str, str]]:
    if name == "current":
        return [("current", "current")]
    if name == "manual":
        return [("manual_exposure", "manual_exposure")]
    if name == "auto":
        return [("auto_exposure", "auto_exposure")]
    return [
        ("current", "current"),
        ("manual_exposure", "manual_exposure"),
        ("auto_exposure", "auto_exposure"),
    ]


def main() -> int:
    args = parse_args()
    device = args.device
    if not Path(device).exists():
        print(f"WARNING: {device} does not exist. Check camera index/device mapping.")

    owner_output, controls_output, kernel_output = print_system_checks(device)
    stats = [
        sample_reads(label, args, profile, args.duration)
        for label, profile in selected_profiles(args.profiles)
    ]

    print()
    print("=== Likely cause ===")
    for idx, reason in enumerate(
        infer_reason(stats, owner_output, controls_output, kernel_output, args),
        start=1,
    ):
        print(f"{idx}. {reason}")

    print()
    print("=== Practical next checks ===")
    print("- Run this test with ROS stopped first.")
    print("- If kernel USB errors appear, change USB cable/port and avoid unpowered hubs.")
    print("- If only auto/manual exposure is slow, keep the faster exposure mode in YAML.")
    print("- If /dev/video0 is busy, kill the process shown by fuser before launching ROS.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
