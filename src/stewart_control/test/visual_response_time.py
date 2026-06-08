#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import statistics
import time
from collections import deque
from pathlib import Path

import matplotlib.pyplot as plt
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

LATENCY_LABELS = {
    "aruco_to_fusion_pose": "ArUco -> F_pose",
    "fusion_pose_to_command": "F_pose -> command",
    "aruco_to_command": "ArUco -> command",
    "command_to_feedback": "Command -> feedback",
    "aruco_to_feedback": "ArUco -> feedback",
}


class RollingSeries:
    def __init__(self, maxlen: int):
        self.times = deque(maxlen=maxlen)
        self.values = deque(maxlen=maxlen)

    def append(self, timestamp: float, value: float) -> None:
        self.times.append(float(timestamp))
        self.values.append(float(value))

    def clear(self) -> None:
        self.times.clear()
        self.values.clear()

    def relative_times(self, now: float) -> list[float]:
        return [timestamp - now for timestamp in self.times]

    def latest(self):
        if not self.values:
            return None
        return self.values[-1]

    def stats(self):
        if not self.values:
            return None
        values = list(self.values)
        return {
            "avg": statistics.fmean(values),
            "min": min(values),
            "max": max(values),
            "latest": values[-1],
        }


class RollingVectorSeries:
    def __init__(self, maxlen: int):
        self.times = deque(maxlen=maxlen)
        self.values = deque(maxlen=maxlen)

    def append(self, timestamp: float, values) -> None:
        self.times.append(float(timestamp))
        self.values.append([float(value) for value in values])

    def relative_times(self, now: float) -> list[float]:
        return [timestamp - now for timestamp in self.times]

    def component(self, index: int) -> list[float]:
        return [row[index] for row in self.values if len(row) > index]

    def latest(self):
        if not self.values:
            return None
        return self.values[-1]


class TopicTiming:
    def __init__(self, maxlen: int):
        self.arrivals = deque(maxlen=maxlen)
        self.period_ms = RollingSeries(maxlen)

    def record(self, timestamp: float) -> None:
        if self.arrivals:
            self.period_ms.append(timestamp, (timestamp - self.arrivals[-1]) * 1000.0)
        self.arrivals.append(timestamp)

    def latest_time(self):
        if not self.arrivals:
            return None
        return self.arrivals[-1]

    def rate_hz(self, now: float, window_s: float) -> float:
        cutoff = now - window_s
        count = sum(1 for timestamp in self.arrivals if timestamp >= cutoff)
        if window_s <= 0.0:
            return 0.0
        return count / window_s

    def stale_s(self, now: float):
        latest = self.latest_time()
        if latest is None:
            return None
        return now - latest


class ResponseTimeNode(Node):
    def __init__(self, maxlen: int):
        super().__init__("visual_response_time")
        self.timings = {name: TopicTiming(maxlen) for name in TOPICS}
        self.latencies = {
            name: RollingSeries(maxlen) for name in LATENCY_LABELS
        }
        self.values = {
            "aruco_position": RollingVectorSeries(maxlen),
            "aruco_orientation": RollingVectorSeries(maxlen),
            "imu": RollingVectorSeries(maxlen),
            "fusion_orientation": RollingVectorSeries(maxlen),
            "fusion_pose": RollingVectorSeries(maxlen),
            "command": RollingVectorSeries(maxlen),
            "feedback": RollingVectorSeries(maxlen),
            "ik_input": RollingVectorSeries(maxlen),
        }
        self.events = []
        self.max_events = maxlen * 8

        self.create_subscription(
            Float32MultiArray,
            TOPICS["aruco_position"],
            lambda msg: self._record_vector_topic("aruco_position", msg.data[:3]),
            1,
        )
        self.create_subscription(
            Float32MultiArray,
            TOPICS["aruco_orientation"],
            lambda msg: self._record_vector_topic("aruco_orientation", msg.data[:3]),
            1,
        )
        self.create_subscription(
            Image,
            TOPICS["camera_image"],
            lambda msg: self._record_topic("camera_image"),
            1,
        )
        self.create_subscription(
            Float32MultiArray,
            TOPICS["imu"],
            lambda msg: self._record_vector_topic("imu", msg.data[:3]),
            1,
        )
        self.create_subscription(
            Float32MultiArray,
            TOPICS["fusion_orientation"],
            lambda msg: self._record_vector_topic("fusion_orientation", msg.data[:3]),
            1,
        )
        self.create_subscription(
            Float32MultiArray,
            TOPICS["fusion_pose"],
            self._record_fusion_pose,
            1,
        )
        self.create_subscription(
            Float32MultiArray,
            TOPICS["command"],
            self._record_command,
            1,
        )
        self.create_subscription(
            Float32MultiArray,
            TOPICS["feedback"],
            self._record_feedback,
            1,
        )

    def _remember_event(self, timestamp: float, topic: str, latency_name=None, value_ms=None):
        self.events.append(
            {
                "time_s": timestamp,
                "topic": topic,
                "latency": "" if latency_name is None else latency_name,
                "latency_ms": "" if value_ms is None else f"{value_ms:.3f}",
            }
        )
        if len(self.events) > self.max_events:
            del self.events[: len(self.events) - self.max_events]

    def _record_topic(self, name: str) -> float:
        now = time.monotonic()
        self.timings[name].record(now)
        self._remember_event(now, TOPICS[name])
        return now

    def _record_vector_topic(self, name: str, values) -> float:
        now = self._record_topic(name)
        self.values[name].append(now, values)
        if name in ("aruco_position", "aruco_orientation"):
            self._update_ik_input_from_aruco(now)
        return now

    def _update_ik_input_from_aruco(self, now: float) -> None:
        if self.timings["fusion_pose"].latest_time() is not None:
            return

        position = self.values["aruco_position"].latest()
        orientation = self.values["aruco_orientation"].latest()
        if position is None or orientation is None:
            return
        if len(position) >= 3 and len(orientation) >= 3:
            self.values["ik_input"].append(now, position[:3] + orientation[:3])

    def _latest_aruco_time(self):
        candidates = [
            self.timings["aruco_position"].latest_time(),
            self.timings["aruco_orientation"].latest_time(),
        ]
        candidates = [value for value in candidates if value is not None]
        if not candidates:
            return None
        return max(candidates)

    def _append_latency(self, name: str, now: float, start_time) -> None:
        if start_time is None:
            return
        latency_ms = max(0.0, (now - start_time) * 1000.0)
        self.latencies[name].append(now, latency_ms)
        self._remember_event(now, TOPICS.get(name, name), name, latency_ms)

    def _record_fusion_pose(self, msg) -> None:
        now = time.monotonic()
        self.timings["fusion_pose"].record(now)
        self._remember_event(now, TOPICS["fusion_pose"])
        # In the current automatic mode, /F_pose is the best externally visible
        # proxy for the pose that stewart_node sends into inverse kinematics.
        # It contains x, y, z in meters and roll, pitch, yaw in degrees.
        pose = list(msg.data[:6])
        if len(pose) >= 6:
            self.values["fusion_pose"].append(now, pose)
            self.values["ik_input"].append(now, pose)
        self._append_latency("aruco_to_fusion_pose", now, self._latest_aruco_time())

    def _record_command(self, msg) -> None:
        now = self._record_vector_topic("command", msg.data[:6])
        fusion_pose_time = self.timings["fusion_pose"].latest_time()
        aruco_time = self._latest_aruco_time()
        self._append_latency("fusion_pose_to_command", now, fusion_pose_time)
        self._append_latency("aruco_to_command", now, aruco_time)

    def _record_feedback(self, msg) -> None:
        now = self._record_vector_topic("feedback", msg.data[:6])
        command_time = self.timings["command"].latest_time()
        aruco_time = self._latest_aruco_time()
        self._append_latency("command_to_feedback", now, command_time)
        self._append_latency("aruco_to_feedback", now, aruco_time)

    def rates(self, now: float, window_s: float):
        return {
            name: timing.rate_hz(now, window_s)
            for name, timing in self.timings.items()
        }

    def write_csv(self, output_path: Path) -> None:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with output_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(
                handle,
                fieldnames=["time_s", "topic", "latency", "latency_ms"],
            )
            writer.writeheader()
            writer.writerows(self.events)


class ResponseTimeDashboard:
    def __init__(self, node: ResponseTimeNode, args: argparse.Namespace):
        self.node = node
        self.args = args
        self.started = time.monotonic()
        self.last_csv_write = 0.0

        plt.style.use("seaborn-v0_8-darkgrid")
        self.fig, self.axes = plt.subplots(3, 2, figsize=(15, 10))
        self.fig.canvas.manager.set_window_title("Stewart Response Time Monitor")
        self.fig.suptitle("Stewart Platform Response Time Monitor")

        self.ax_sensor_period = self.axes[0][0]
        self.ax_latency = self.axes[0][1]
        self.ax_ik_input = self.axes[1][0]
        self.ax_command = self.axes[1][1]
        self.ax_feedback = self.axes[2][0]
        self.ax_summary = self.axes[2][1]
        self.fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.95))

    def update(self) -> None:
        redraw_at = time.monotonic() + self.args.refresh
        while (
            rclpy.ok()
            and plt.fignum_exists(self.fig.number)
            and time.monotonic() < redraw_at
        ):
            rclpy.spin_once(self.node, timeout_sec=0.001)
            plt.pause(0.001)

        now = time.monotonic()
        self._draw_sensor_periods(now)
        self._draw_latencies(now)
        self._draw_ik_input(now)
        self._draw_motor_values(now)
        self._draw_summary(now)
        self._write_csv_if_due(now)
        plt.pause(0.001)

    def _draw_sensor_periods(self, now: float) -> None:
        self.ax_sensor_period.clear()
        for name in ("camera_image", "aruco_position", "aruco_orientation", "imu"):
            series = self.node.timings[name].period_ms
            if not series.values:
                continue
            self.ax_sensor_period.plot(
                series.relative_times(now),
                list(series.values),
                label=TOPICS[name],
                linewidth=1.6,
            )

        self.ax_sensor_period.set_title("Camera / ArUco / IMU Detection Timing")
        self.ax_sensor_period.set_xlabel("Seconds from now")
        self.ax_sensor_period.set_ylabel("ms between messages")
        self.ax_sensor_period.set_xlim(-self.args.window, 0)
        self.ax_sensor_period.set_ylim(bottom=0)
        self.ax_sensor_period.legend(loc="upper left", fontsize=8)

    def _draw_latencies(self, now: float) -> None:
        self.ax_latency.clear()
        for name, series in self.node.latencies.items():
            if not series.values:
                continue
            self.ax_latency.plot(
                series.relative_times(now),
                list(series.values),
                label=LATENCY_LABELS[name],
                linewidth=1.8,
            )

        self.ax_latency.set_title("Estimated Stage Latency")
        self.ax_latency.set_xlabel("Seconds from now")
        self.ax_latency.set_ylabel("ms")
        self.ax_latency.set_xlim(-self.args.window, 0)
        self.ax_latency.set_ylim(bottom=0)
        self.ax_latency.legend(loc="upper left", fontsize=8)

    def _draw_ik_input(self, now: float) -> None:
        self.ax_ik_input.clear()
        series = self.node.values["ik_input"]
        if series.values:
            rel_times = series.relative_times(now)
            labels = ("X cm", "Y cm", "Z cm", "Roll deg", "Pitch deg", "Yaw deg")
            for index, label in enumerate(labels):
                values = series.component(index)
                if not values:
                    continue
                if index < 3:
                    values = [value * 100.0 for value in values]
                self.ax_ik_input.plot(
                    rel_times[-len(values):],
                    values,
                    label=label,
                    linewidth=1.5,
                    linestyle="-" if index < 3 else "--",
                )

        self.ax_ik_input.set_title("Pose Going Toward IK (/F_pose or ArUco fallback)")
        self.ax_ik_input.set_xlabel("Seconds from now")
        self.ax_ik_input.set_ylabel("cm / deg")
        self.ax_ik_input.set_xlim(-self.args.window, 0)
        self.ax_ik_input.legend(loc="upper left", fontsize=8, ncol=2)

    def _draw_motor_values(self, now: float) -> None:
        self._draw_six_motor_series(
            self.ax_command,
            self.node.values["command"],
            "IK Output / Command Sent to Arduino",
            now,
        )
        self._draw_six_motor_series(
            self.ax_feedback,
            self.node.values["feedback"],
            "Arduino Feedback",
            now,
        )

    def _draw_six_motor_series(self, axis, series: RollingVectorSeries, title: str, now: float):
        axis.clear()
        if series.values:
            rel_times = series.relative_times(now)
            for index in range(6):
                values = series.component(index)
                if not values:
                    continue
                axis.plot(
                    rel_times[-len(values):],
                    values,
                    label=f"M{index + 1}",
                    linewidth=1.3,
                )
        axis.set_title(title)
        axis.set_xlabel("Seconds from now")
        axis.set_ylabel("cm")
        axis.set_xlim(-self.args.window, 0)
        axis.legend(loc="upper left", fontsize=8, ncol=3)

    def _draw_summary(self, now: float) -> None:
        self.ax_summary.clear()
        self.ax_summary.axis("off")

        lines = [
            f"Running: {now - self.started:6.1f} s",
            "",
            f"Rates over {self.args.rate_window:.1f}s",
        ]

        rates = self.node.rates(now, self.args.rate_window)
        for name in (
            "camera_image",
            "aruco_position",
            "aruco_orientation",
            "imu",
            "fusion_pose",
            "command",
            "feedback",
        ):
            lines.append(f"{TOPICS[name]:<24} {rates[name]:6.1f} Hz")

        lines.extend([
            "",
            "Latest / Average Latency",
        ])

        for name, label in LATENCY_LABELS.items():
            stats = self.node.latencies[name].stats()
            if stats is None:
                lines.append(f"{label:<22} waiting")
                continue
            lines.append(
                f"{label:<22} latest {stats['latest']:7.2f} ms"
                f"   avg {stats['avg']:7.2f} ms"
                f"   max {stats['max']:7.2f} ms"
            )

        lines.extend(["", "Topic Freshness"])
        for name, topic in TOPICS.items():
            stale = self.node.timings[name].stale_s(now)
            if stale is None:
                lines.append(f"{topic:<24} no data")
            else:
                state = "STALE" if stale > self.args.stale_after else "ok"
                lines.append(f"{topic:<24} {stale:6.2f} s ago  {state}")

        self.ax_summary.text(
            0.01,
            0.98,
            "\n".join(lines),
            va="top",
            ha="left",
            family="monospace",
            fontsize=9,
        )

    def _write_csv_if_due(self, now: float) -> None:
        if self.args.csv is None:
            return
        if (now - self.last_csv_write) < self.args.csv_interval:
            return
        self.node.write_csv(Path(self.args.csv))
        self.last_csv_write = now

                  
def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Visualize ROS topic rates and estimated response times for the "
            "Stewart platform. Run this while the normal nodes are already running."
        )
    )
    parser.add_argument(
        "--window",
        type=float,
        default=20.0,
        help="Time span shown in the line graphs, in seconds.",
    )
    parser.add_argument(
        "--rate-window",
        type=float,
        default=5.0,
        help="Rolling time span used for Hz bars, in seconds.",
    )
    parser.add_argument(
        "--refresh",
        type=float,
        default=0.10,
        help="Graph refresh interval in seconds.",
    )
    parser.add_argument(
        "--max-samples",
        type=int,
        default=1200,
        help="Maximum samples kept in memory per series.",
    )
    parser.add_argument(
        "--stale-after",
        type=float,
        default=1.0,
        help="Mark a topic stale when no message has arrived for this many seconds.",
    )
    parser.add_argument(
        "--csv",
        type=str,
        default=None,
        help="Optional CSV path for saving raw timing events.",
    )
    parser.add_argument(
        "--csv-interval",
        type=float,
        default=2.0,
        help="How often to update the CSV file when --csv is set.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.window <= 0.0:
        raise SystemExit("--window must be greater than 0")
    if args.rate_window <= 0.0:
        raise SystemExit("--rate-window must be greater than 0")
    if args.refresh <= 0.0:
        raise SystemExit("--refresh must be greater than 0")
    if args.max_samples <= 1:
        raise SystemExit("--max-samples must be greater than 1")

    rclpy.init()
    node = ResponseTimeNode(args.max_samples)
    dashboard = ResponseTimeDashboard(node, args)

    print("Visual response-time monitor started.")
    print("Run your acquisition and motion nodes in another terminal.")
    print("Close the graph window or press Ctrl+C to stop.")

    try:
        while rclpy.ok() and plt.fignum_exists(dashboard.fig.number):
            dashboard.update()
    except KeyboardInterrupt:
        pass
    finally:
        if args.csv is not None:
            node.write_csv(Path(args.csv))
            print(f"Saved timing CSV: {args.csv}")
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
