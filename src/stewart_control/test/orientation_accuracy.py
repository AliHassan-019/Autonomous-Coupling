#!/usr/bin/env python3

from __future__ import annotations

import argparse
import time
from collections import deque

import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


OK_FILL = "#d8f3dc"
OK_LINE = "#2d6a4f"
COLORS = {
    "camera_imu": "#1f77b4",
    "fusion_camera": "#2ca02c",
    "fusion_imu": "#d62728",
}


def wrap_deg(angle: float) -> float:
    return (float(angle) + 180.0) % 360.0 - 180.0


def angle_error(a, b):
    return [wrap_deg(float(x) - float(y)) for x, y in zip(a, b)]


def smooth(previous, current, alpha):
    if previous is None:
        return [float(value) for value in current]
    return [
        (1.0 - alpha) * float(old) + alpha * float(new)
        for old, new in zip(previous, current)
    ]


class RollingSeries:
    def __init__(self, maxlen):
        self.times = deque(maxlen=maxlen)
        self.values = deque(maxlen=maxlen)

    def append(self, timestamp, values):
        self.times.append(float(timestamp))
        self.values.append([float(value) for value in values])

    def relative_times(self, now):
        return [timestamp - now for timestamp in self.times]

    def component(self, index):
        return [row[index] for row in self.values if len(row) > index]

    def latest(self):
        if not self.values:
            return None
        return self.values[-1]

    def recent_max_abs(self, count=100):
        if not self.values:
            return None
        recent = list(self.values)[-min(count, len(self.values)) :]
        return max(abs(value) for row in recent for value in row)


class OrientationAccuracyNode(Node):
    def __init__(self, maxlen, alpha):
        super().__init__("orientation_accuracy")
        self.alpha = float(alpha)
        self.latest = {}
        self.smoothed_errors = {
            "camera_imu": None,
            "fusion_camera": None,
            "fusion_imu": None,
        }
        self.series = {
            "camera_imu": RollingSeries(maxlen),
            "fusion_camera": RollingSeries(maxlen),
            "fusion_imu": RollingSeries(maxlen),
        }

        self.create_subscription(
            Float32MultiArray, "aruco_orientation", self._callback("camera"), 10
        )
        self.create_subscription(Float32MultiArray, "imu_error", self._callback("imu"), 10)
        self.create_subscription(
            Float32MultiArray, "F_orientation", self._callback("fusion"), 10
        )

    def _callback(self, name):
        def callback(msg):
            if len(msg.data) < 3:
                return
            now = time.monotonic()
            self.latest[name] = [float(value) for value in msg.data[:3]]
            self._update_errors(now)

        return callback

    def _append_error(self, name, now, raw_error):
        smoothed_error = smooth(self.smoothed_errors[name], raw_error, self.alpha)
        self.smoothed_errors[name] = smoothed_error
        self.series[name].append(now, smoothed_error)

    def _update_errors(self, now):
        camera = self.latest.get("camera")
        imu = self.latest.get("imu")
        fusion = self.latest.get("fusion")

        if camera is not None and imu is not None:
            self._append_error("camera_imu", now, angle_error(camera, imu))
        if fusion is not None and camera is not None:
            self._append_error("fusion_camera", now, angle_error(fusion, camera))
        if fusion is not None and imu is not None:
            self._append_error("fusion_imu", now, angle_error(fusion, imu))


class AccuracyPlot:
    AXES = ("Roll", "Pitch", "Yaw")

    def __init__(self, node, window_s, ok_deg):
        self.node = node
        self.window_s = float(window_s)
        self.ok_deg = float(ok_deg)
        self.fig, all_axes = plt.subplots(
            4,
            1,
            figsize=(11, 8.5),
            gridspec_kw={"height_ratios": [1.0, 1.0, 1.0, 0.35]},
        )
        self.axes = all_axes[:3]
        self.table_ax = all_axes[3]
        self.table_ax.axis("off")
        self.fig.canvas.manager.set_window_title("Orientation Accuracy")
        self.lines = {}
        self._build()

    def _build(self):
        self.fig.suptitle("Orientation Accuracy", fontsize=14, fontweight="bold")
        for index, ax in enumerate(self.axes):
            ax.set_title(f"{self.AXES[index]} Error")
            ax.set_ylabel("deg")
            ax.grid(True, alpha=0.35)
            ax.axhspan(-self.ok_deg, self.ok_deg, color=OK_FILL, alpha=0.7)
            ax.axhline(0.0, color="black", linewidth=0.8, alpha=0.5)
            ax.axhline(self.ok_deg, color=OK_LINE, linestyle="--", linewidth=0.9)
            ax.axhline(-self.ok_deg, color=OK_LINE, linestyle="--", linewidth=0.9)
            for series_name, label in (
                ("camera_imu", "Camera - IMU"),
                ("fusion_camera", "Fusion - Camera"),
                ("fusion_imu", "Fusion - IMU"),
            ):
                (line,) = ax.plot(
                    [],
                    [],
                    linewidth=1.8,
                    label=label,
                    color=COLORS[series_name],
                )
                self.lines[(series_name, index)] = line
            ax.legend(loc="upper left", fontsize=9)
            ax.set_xlim(-self.window_s, 0.0)
        self.axes[-1].set_xlabel("seconds ago")
        self.fig.subplots_adjust(top=0.92, bottom=0.06, hspace=0.58)

    @staticmethod
    def _fmt_axis(values, index):
        if values is None or len(values) <= index:
            return "--"
        return f"{values[index]:+.2f}"

    @staticmethod
    def _status(value, limit):
        if value is None:
            return "WAIT"
        return "OK" if value <= limit else "WATCH"

    def _update_table(self):
        self.table_ax.clear()
        self.table_ax.axis("off")

        camera_imu = self.node.series["camera_imu"].latest()
        fusion_camera = self.node.series["fusion_camera"].latest()
        fusion_imu = self.node.series["fusion_imu"].latest()
        camera_imu_max = self.node.series["camera_imu"].recent_max_abs()
        fusion_camera_max = self.node.series["fusion_camera"].recent_max_abs()
        fusion_imu_max = self.node.series["fusion_imu"].recent_max_abs()

        rows = [
            [
                "Camera - IMU",
                self._fmt_axis(camera_imu, 0),
                self._fmt_axis(camera_imu, 1),
                self._fmt_axis(camera_imu, 2),
                self._status(camera_imu_max, self.ok_deg),
            ],
            [
                "Fusion - Camera",
                self._fmt_axis(fusion_camera, 0),
                self._fmt_axis(fusion_camera, 1),
                self._fmt_axis(fusion_camera, 2),
                self._status(fusion_camera_max, self.ok_deg),
            ],
            [
                "Fusion - IMU",
                self._fmt_axis(fusion_imu, 0),
                self._fmt_axis(fusion_imu, 1),
                self._fmt_axis(fusion_imu, 2),
                self._status(fusion_imu_max, self.ok_deg),
            ],
        ]

        table = self.table_ax.table(
            cellText=rows,
            colLabels=("Comparison", "Roll deg", "Pitch deg", "Yaw deg", "Status"),
            cellLoc="center",
            colLoc="center",
            loc="center",
            bbox=[0.08, 0.08, 0.84, 0.84],
        )
        table.auto_set_font_size(False)
        table.set_fontsize(9)

        for (row, col), cell in table.get_celld().items():
            if row == 0:
                cell.set_text_props(weight="bold")
                cell.set_facecolor("#eeeeee")
            elif col == 4:
                status = cell.get_text().get_text()
                if status == "OK":
                    cell.set_facecolor("#d8f3dc")
                elif status == "WATCH":
                    cell.set_facecolor("#ffe5d0")
                elif status == "WAIT":
                    cell.set_facecolor("#fff3cd")

    def update(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)
        now = time.monotonic()

        for (series_name, index), line in self.lines.items():
            series = self.node.series[series_name]
            x_values = series.relative_times(now)
            y_values = series.component(index)
            count = min(len(x_values), len(y_values))
            line.set_data(x_values[-count:], y_values[-count:])

        for ax in self.axes:
            ax.set_xlim(-self.window_s, 0.0)
            ax.relim()
            ax.autoscale_view(scalex=False, scaley=True)

        self._update_table()
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()


def parse_args():
    parser = argparse.ArgumentParser(
        description="Smooth accuracy graph comparing camera, IMU, and fusion orientation."
    )
    parser.add_argument("--window", type=float, default=20.0)
    parser.add_argument("--refresh", type=float, default=0.05)
    parser.add_argument("--samples", type=int, default=2500)
    parser.add_argument(
        "--smooth-alpha",
        type=float,
        default=0.18,
        help="Smoothing factor from 0 to 1. Higher is more responsive, lower is smoother.",
    )
    parser.add_argument(
        "--ok-deg",
        type=float,
        default=2.0,
        help="Acceptable orientation accuracy error in degrees.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = OrientationAccuracyNode(maxlen=args.samples, alpha=args.smooth_alpha)
    plot = AccuracyPlot(node=node, window_s=args.window, ok_deg=args.ok_deg)

    plt.ion()
    plt.show(block=False)
    try:
        while plt.fignum_exists(plot.fig.number):
            plot.update()
            plt.pause(args.refresh)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
