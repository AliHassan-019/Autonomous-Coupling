#!/usr/bin/env python3
"""Runtime performance monitor for Stewart platform tests.

The script subscribes to the main ROS topics while also sampling Linux /proc
statistics for relevant processes. It writes a CSV timeline and a JSON summary
that can be used for tuning camera rate, GUI plotting, fusion freshness, and
serial/control loop responsiveness.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import statistics
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray


TOPICS = {
    "aruco_position": "/aruco_position",
    "aruco_orientation": "/aruco_orientation",
    "camera_image": "/camera/image_raw",
    "imu": "/imu_error",
    "fusion_orientation": "/F_orientation",
    "fusion_pose": "/F_pose",
    "command": "/stewart/longueurs",
    "feedback": "/feedback_motors",
}

PROCESS_KEYWORDS = (
    "aruco_node",
    "imu_node",
    "fusion_node",
    "stewart_node",
    "manual_stewart_node",
    "interface_node",
    "posHome_node",
    "ros2",
)
EXCLUDED_PROCESS_KEYWORDS = (
    "vscode",
    "pylance",
    "code ",
    "performance_monitor.py",
)


def percentile(values: list[float], pct: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = int(round((len(ordered) - 1) * pct / 100.0))
    return ordered[index]


def mean(values: list[float]) -> float | None:
    if not values:
        return None
    return statistics.fmean(values)


@dataclass
class TopicStats:
    arrivals: deque[float] = field(default_factory=lambda: deque(maxlen=5000))
    periods_ms: deque[float] = field(default_factory=lambda: deque(maxlen=5000))
    payload_sizes: deque[int] = field(default_factory=lambda: deque(maxlen=5000))

    def record(self, timestamp: float, payload_size: int = 0) -> None:
        if self.arrivals:
            self.periods_ms.append((timestamp - self.arrivals[-1]) * 1000.0)
        self.arrivals.append(timestamp)
        self.payload_sizes.append(int(payload_size))

    def rate_hz(self, now: float, window_s: float) -> float:
        if window_s <= 0.0:
            return 0.0
        cutoff = now - window_s
        return sum(1 for value in self.arrivals if value >= cutoff) / window_s

    def stale_s(self, now: float) -> float | None:
        if not self.arrivals:
            return None
        return now - self.arrivals[-1]


@dataclass
class ProcessSample:
    pid: int
    name: str
    cpu_pct: float
    rss_mb: float


class ProcSampler:
    """Samples CPU and RSS from /proc without external dependencies."""

    def __init__(self, keywords: tuple[str, ...]):
        self.keywords = tuple(item.lower() for item in keywords)
        self.clock_ticks = os.sysconf(os.sysconf_names["SC_CLK_TCK"])
        self.page_size = os.sysconf(os.sysconf_names["SC_PAGE_SIZE"])
        self.previous: dict[int, tuple[float, int]] = {}

    @staticmethod
    def _read_text(path: Path) -> str | None:
        try:
            return path.read_text(encoding="utf-8", errors="replace")
        except OSError:
            return None

    def _matches(self, pid_dir: Path) -> tuple[bool, str]:
        command = self._read_text(pid_dir / "cmdline")
        if command:
            name = command.replace("\x00", " ").strip()
        else:
            name = self._read_text(pid_dir / "comm") or ""
            name = name.strip()

        lowered = name.lower()
        if any(keyword in lowered for keyword in EXCLUDED_PROCESS_KEYWORDS):
            return False, name
        return any(keyword in lowered for keyword in self.keywords), name

    def sample(self) -> list[ProcessSample]:
        now = time.monotonic()
        rows: list[ProcessSample] = []

        for pid_dir in Path("/proc").iterdir():
            if not pid_dir.name.isdigit():
                continue

            matched, name = self._matches(pid_dir)
            if not matched:
                continue

            stat = self._read_text(pid_dir / "stat")
            statm = self._read_text(pid_dir / "statm")
            if not stat or not statm:
                continue

            try:
                parts = stat.split()
                utime = int(parts[13])
                stime = int(parts[14])
                total_ticks = utime + stime
                rss_pages = int(statm.split()[1])
                pid = int(pid_dir.name)
            except (IndexError, ValueError):
                continue

            cpu_pct = 0.0
            previous = self.previous.get(pid)
            if previous is not None:
                prev_time, prev_ticks = previous
                elapsed = now - prev_time
                if elapsed > 0.0:
                    cpu_pct = (
                        (total_ticks - prev_ticks)
                        / self.clock_ticks
                        / elapsed
                        * 100.0
                    )
            self.previous[pid] = (now, total_ticks)

            rss_mb = rss_pages * self.page_size / (1024.0 * 1024.0)
            rows.append(ProcessSample(pid=pid, name=name[:120], cpu_pct=cpu_pct, rss_mb=rss_mb))

        return rows


class PerformanceMonitor(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("stewart_performance_monitor")
        self.args = args
        self.started = time.monotonic()
        self.stats = {name: TopicStats() for name in TOPICS}
        self.command_times = deque(maxlen=2000)
        self.feedback_latencies_ms = deque(maxlen=2000)
        self.pending_command_time: float | None = None
        self.proc_sampler = ProcSampler(PROCESS_KEYWORDS)

        self._subscribe_vector("aruco_position", self._record_vector("aruco_position"))
        self._subscribe_vector(
            "aruco_orientation", self._record_vector("aruco_orientation")
        )
        self._subscribe_vector("imu", self._record_vector("imu"))
        self._subscribe_vector(
            "fusion_orientation", self._record_vector("fusion_orientation")
        )
        self._subscribe_vector("fusion_pose", self._record_vector("fusion_pose"))
        self._subscribe_vector("command", self._record_command)
        self._subscribe_vector("feedback", self._record_feedback)
        self.create_subscription(Image, TOPICS["camera_image"], self._record_image, 1)

    def _subscribe_vector(self, name: str, callback: Callable) -> None:
        self.create_subscription(Float32MultiArray, TOPICS[name], callback, 1)

    def _record_vector(self, name: str) -> Callable:
        def callback(msg: Float32MultiArray) -> None:
            now = time.monotonic()
            self.stats[name].record(now, len(msg.data))

        return callback

    def _record_command(self, msg: Float32MultiArray) -> None:
        now = time.monotonic()
        self.stats["command"].record(now, len(msg.data))
        self.command_times.append(now)
        self.pending_command_time = now

    def _record_feedback(self, msg: Float32MultiArray) -> None:
        now = time.monotonic()
        self.stats["feedback"].record(now, len(msg.data))
        if self.pending_command_time is not None:
            self.feedback_latencies_ms.append((now - self.pending_command_time) * 1000.0)
            self.pending_command_time = None

    def _record_image(self, msg: Image) -> None:
        now = time.monotonic()
        self.stats["camera_image"].record(now, len(msg.data))

    def row(self) -> dict[str, object]:
        now = time.monotonic()
        processes = self.proc_sampler.sample()
        cpu_total = sum(item.cpu_pct for item in processes)
        rss_total = sum(item.rss_mb for item in processes)
        top_cpu = max(processes, key=lambda item: item.cpu_pct, default=None)
        top_mem = max(processes, key=lambda item: item.rss_mb, default=None)

        row: dict[str, object] = {
            "elapsed_s": round(now - self.started, 3),
            "proc_count": len(processes),
            "cpu_total_pct": round(cpu_total, 2),
            "rss_total_mb": round(rss_total, 2),
            "top_cpu": "" if top_cpu is None else f"{top_cpu.pid}:{top_cpu.cpu_pct:.1f}:{top_cpu.name}",
            "top_mem": "" if top_mem is None else f"{top_mem.pid}:{top_mem.rss_mb:.1f}:{top_mem.name}",
            "feedback_after_command_ms_latest": "",
            "feedback_after_command_ms_avg": "",
            "feedback_after_command_ms_p95": "",
        }

        latency_values = list(self.feedback_latencies_ms)
        if latency_values:
            row["feedback_after_command_ms_latest"] = round(latency_values[-1], 2)
            row["feedback_after_command_ms_avg"] = round(mean(latency_values), 2)
            row["feedback_after_command_ms_p95"] = round(percentile(latency_values, 95), 2)

        for name, stats in self.stats.items():
            row[f"{name}_rate_hz"] = round(stats.rate_hz(now, self.args.rate_window_s), 2)
            stale = stats.stale_s(now)
            row[f"{name}_stale_s"] = "" if stale is None else round(stale, 3)
            periods = list(stats.periods_ms)
            row[f"{name}_period_avg_ms"] = (
                "" if not periods else round(mean(periods[-200:]), 2)
            )
            row[f"{name}_period_p95_ms"] = (
                "" if not periods else round(percentile(periods[-200:], 95), 2)
            )
            payloads = list(stats.payload_sizes)
            row[f"{name}_payload_avg_bytes"] = (
                "" if not payloads else round(mean(payloads[-200:]), 1)
            )

        return row

    def summary(self) -> dict[str, object]:
        now = time.monotonic()
        topics = {}
        for name, stats in self.stats.items():
            periods = list(stats.periods_ms)
            topics[name] = {
                "messages": len(stats.arrivals),
                "rate_hz_recent": round(stats.rate_hz(now, self.args.rate_window_s), 3),
                "stale_s": stats.stale_s(now),
                "period_avg_ms": mean(periods),
                "period_p95_ms": percentile(periods, 95),
            }

        latencies = list(self.feedback_latencies_ms)
        return {
            "duration_s": round(now - self.started, 3),
            "rate_window_s": self.args.rate_window_s,
            "topics": topics,
            "feedback_after_command_ms": {
                "count": len(latencies),
                "avg": mean(latencies),
                "p95": percentile(latencies, 95),
                "max": max(latencies) if latencies else None,
            },
        }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Monitor Stewart ROS topic rates, stale data, CPU, memory, and command/feedback timing."
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=60.0,
        help="Monitoring duration in seconds. Use 0 for until Ctrl+C.",
    )
    parser.add_argument(
        "--sample-period",
        type=float,
        default=1.0,
        help="CSV sample period in seconds.",
    )
    parser.add_argument(
        "--rate-window-s",
        type=float,
        default=5.0,
        help="Rolling window used for topic rate calculations.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("performance_logs"),
        help="Directory where CSV and summary JSON are written.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    csv_path = args.output_dir / f"performance_{stamp}.csv"
    summary_path = args.output_dir / f"performance_{stamp}_summary.json"

    rclpy.init()
    node = PerformanceMonitor(args)
    fieldnames = list(node.row().keys())
    deadline = None if args.duration <= 0.0 else time.monotonic() + args.duration
    next_sample = time.monotonic()

    print(f"Writing samples to: {csv_path}")
    print(f"Writing summary to: {summary_path}")
    print("Press Ctrl+C to stop early.")

    try:
        with csv_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()

            while rclpy.ok():
                if deadline is not None and time.monotonic() >= deadline:
                    break

                rclpy.spin_once(node, timeout_sec=0.05)
                now = time.monotonic()
                if now >= next_sample:
                    row = node.row()
                    writer.writerow(row)
                    handle.flush()
                    print(
                        "t={elapsed_s:>6}s cpu={cpu_total_pct:>5}% mem={rss_total_mb:>7}MB "
                        "cam={camera_image_rate_hz:>5}Hz aruco={aruco_position_rate_hz:>5}Hz "
                        "imu={imu_rate_hz:>5}Hz cmd={command_rate_hz:>5}Hz fb={feedback_rate_hz:>5}Hz".format(
                            **row
                        )
                    )
                    next_sample = now + args.sample_period
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        summary_path.write_text(
            json.dumps(node.summary(), indent=2),
            encoding="utf-8",
        )
        node.destroy_node()
        rclpy.shutdown()

    print(f"Done. CSV: {csv_path}")
    print(f"Done. Summary: {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
