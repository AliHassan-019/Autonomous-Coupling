#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt


LATENCY_LABELS = {
    "aruco_to_fusion_pose": "ArUco -> F_pose",
    "fusion_pose_to_command": "F_pose -> Command",
    "aruco_to_command": "ArUco -> Command",
    "command_to_feedback": "Command -> Feedback",
    "aruco_to_feedback": "ArUco -> Feedback",
}

POSITION_TARGET_COLUMNS = (
    ("target_x", "measured_x"),
    ("target_y", "measured_y"),
    ("target_z", "measured_z"),
)
ORIENTATION_TARGET_COLUMNS = (
    ("target_roll", "measured_roll"),
    ("target_pitch", "measured_pitch"),
    ("target_yaw", "measured_yaw"),
)
ORIENTATION_TABLE_ORDER = ["Camera-IMU", "Fusion-Camera", "Fusion-IMU"]
ORIENTATION_TABLE_AXES = ("roll", "pitch", "yaw")


def read_csv_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def parse_float(value) -> float | None:
    if value is None:
        return None
    value = str(value).strip()
    if value == "":
        return None
    try:
        result = float(value)
    except ValueError:
        return None
    if not math.isfinite(result):
        return None
    return result


def mean(values: list[float]) -> float:
    if not values:
        return float("nan")
    return sum(values) / len(values)


def percentile(values: list[float], pct: float) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    rank = (len(ordered) - 1) * pct / 100.0
    lower = int(math.floor(rank))
    upper = int(math.ceil(rank))
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (rank - lower)


def angle_error_deg(target: float, measured: float) -> float:
    return abs((measured - target + 180.0) % 360.0 - 180.0)


def summarize_time_csv(path: Path) -> dict[str, dict[str, float]]:
    values_by_latency: dict[str, list[float]] = defaultdict(list)
    for row in read_csv_rows(path):
        latency_name = row.get("latency", "").strip()
        value_ms = parse_float(row.get("latency_ms"))
        if latency_name and value_ms is not None:
            values_by_latency[latency_name].append(value_ms)

    return {
        latency_name: {
            "mean": mean(values),
            "p95": percentile(values, 95),
            "max": max(values) if values else float("nan"),
            "count": float(len(values)),
        }
        for latency_name, values in values_by_latency.items()
    }


def row_metric_name(row: dict[str, str], fallback: str) -> str:
    for key in ("metric", "name", "sensor", "sensor_pair", "axis", "label"):
        value = row.get(key)
        if value is not None and value.strip():
            return value.strip()
    return fallback


def summarize_accuracy_csv(path: Path) -> dict[str, dict[str, float]]:
    values_by_metric: dict[str, list[float]] = defaultdict(list)

    for index, row in enumerate(read_csv_rows(path), start=1):
        metric = row_metric_name(row, f"sample_{index}")

        for key in ("error", "error_deg", "error_cm", "absolute_error", "rmse"):
            value = parse_float(row.get(key))
            if value is not None:
                values_by_metric[metric].append(abs(value))
                break

        for target_key, measured_key in POSITION_TARGET_COLUMNS:
            target = parse_float(row.get(target_key))
            measured = parse_float(row.get(measured_key))
            if target is not None and measured is not None:
                axis = target_key.replace("target_", "").upper()
                values_by_metric[f"Position {axis}"].append(abs(measured - target))

        for target_key, measured_key in ORIENTATION_TARGET_COLUMNS:
            target = parse_float(row.get(target_key))
            measured = parse_float(row.get(measured_key))
            if target is not None and measured is not None:
                axis = target_key.replace("target_", "").title()
                values_by_metric[f"Orientation {axis}"].append(
                    angle_error_deg(target, measured)
                )

        for key, value in row.items():
            if key is None:
                continue
            lowered = key.lower()
            if "error" not in lowered:
                continue
            parsed = parse_float(value)
            if parsed is not None:
                values_by_metric[key].append(abs(parsed))

    return {
        metric: {
            "mean": mean(values),
            "p95": percentile(values, 95),
            "max": max(values) if values else float("nan"),
            "count": float(len(values)),
        }
        for metric, values in values_by_metric.items()
    }


def canonical_comparison_name(value: str) -> str:
    lowered = value.strip().lower().replace("_", "-").replace(" ", "-")
    aliases = {
        "camera-imu": "Camera-IMU",
        "imu-camera": "Camera-IMU",
        "fusion-camera": "Fusion-Camera",
        "camera-fusion": "Fusion-Camera",
        "fusion-imu": "Fusion-IMU",
        "imu-fusion": "Fusion-IMU",
    }
    return aliases.get(lowered, value.strip())


def summarize_orientation_difference_csv(
    path: Path,
) -> dict[str, dict[str, float]] | None:
    values_by_metric: dict[str, dict[str, list[float]]] = defaultdict(
        lambda: {axis: [] for axis in ORIENTATION_TABLE_AXES}
    )

    for row in read_csv_rows(path):
        comparison = None
        for key in ("comparison", "metric", "name", "sensor_pair", "pair", "label"):
            value = row.get(key)
            if value is not None and value.strip():
                comparison = canonical_comparison_name(value)
                break
        if comparison is None:
            continue

        axis_found = False
        for axis in ORIENTATION_TABLE_AXES:
            candidates = (
                axis,
                f"{axis}_deg",
                f"{axis}_error",
                f"{axis}_error_deg",
                f"mean_{axis}",
                f"mean_{axis}_deg",
            )
            for key in candidates:
                parsed = parse_float(row.get(key))
                if parsed is not None:
                    values_by_metric[comparison][axis].append(parsed)
                    axis_found = True
                    break
        if axis_found:
            continue

        axis_label = row.get("axis", "").strip().lower()
        value = parse_float(row.get("error_deg"))
        if axis_label in ORIENTATION_TABLE_AXES and value is not None:
            values_by_metric[comparison][axis_label].append(value)

    if not values_by_metric:
        return None

    summary: dict[str, dict[str, float]] = {}
    for comparison, axis_values in values_by_metric.items():
        if not any(axis_values.values()):
            continue
        summary[comparison] = {
            axis: mean(values) if values else float("nan")
            for axis, values in axis_values.items()
        }
    return summary or None


def improvement_percent(before: float, now: float) -> float:
    if not math.isfinite(before) or not math.isfinite(now) or abs(before) < 1e-12:
        return float("nan")
    return (before - now) / before * 100.0


def common_order(before: dict, now: dict, preferred: list[str] | None = None) -> list[str]:
    names = [name for name in (preferred or []) if name in before or name in now]
    remaining = sorted((set(before) | set(now)) - set(names))
    return names + remaining


def plot_comparison(
    before: dict[str, dict[str, float]],
    now: dict[str, dict[str, float]],
    output_path: Path,
    title: str,
    ylabel: str,
    preferred_order: list[str] | None = None,
    annotation_mode: str = "improvement",
) -> None:
    metrics = common_order(before, now, preferred_order)
    if not metrics:
        raise ValueError(f"No plottable data found for {title}.")

    before_values = [before.get(metric, {}).get("mean", float("nan")) for metric in metrics]
    now_values = [now.get(metric, {}).get("mean", float("nan")) for metric in metrics]
    x_values = list(range(len(metrics)))
    width = 0.38
    finite_values = [
        value for value in before_values + now_values if math.isfinite(value)
    ]
    max_value = max(finite_values) if finite_values else 1.0
    value_label_offset = max(max_value * 0.015, 0.05)
    pair_label_offset = max(max_value * 0.12, 0.2)

    plt.style.use("seaborn-v0_8-darkgrid")
    fig, ax = plt.subplots(figsize=(max(10, len(metrics) * 1.2), 6))
    ax.bar([x - width / 2 for x in x_values], before_values, width, label="Before")
    ax.bar([x + width / 2 for x in x_values], now_values, width, label="Now")

    for x, value in zip([x - width / 2 for x in x_values], before_values):
        if math.isfinite(value):
            ax.text(
                x,
                value + value_label_offset,
                f"{value:.2f}",
                ha="center",
                va="bottom",
                fontsize=8,
            )

    for x, value in zip([x + width / 2 for x in x_values], now_values):
        if math.isfinite(value):
            ax.text(
                x,
                value + value_label_offset,
                f"{value:.2f}",
                ha="center",
                va="bottom",
                fontsize=8,
            )

    for x, before_value, now_value in zip(x_values, before_values, now_values):
        y = max(before_value, now_value)
        if annotation_mode == "value":
            continue
        else:
            change = improvement_percent(before_value, now_value)
            if not math.isfinite(change):
                continue
            label = f"{change:+.1f}%"
        ax.text(
            x,
            y + pair_label_offset,
            label,
            ha="center",
            va="bottom",
            fontsize=9,
            fontweight="bold",
        )

    ax.set_title(title)
    ax.set_ylabel(ylabel)
    ax.set_xticks(x_values)
    ax.set_xticklabels(
        [LATENCY_LABELS.get(metric, metric) for metric in metrics],
        rotation=25,
        ha="right",
    )
    ax.legend()
    ax.set_ylim(0, max_value + pair_label_offset * 1.45)
    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def plot_orientation_difference_bars(
    before: dict[str, dict[str, float]],
    now: dict[str, dict[str, float]],
    output_path: Path,
) -> None:
    axes_order = [("roll", "Roll"), ("pitch", "Pitch"), ("yaw", "Yaw")]
    comparison = "Fusion-IMU" if "Fusion-IMU" in before or "Fusion-IMU" in now else None
    if comparison is None:
        comparison = common_order(before, now, ORIENTATION_TABLE_ORDER)[0]

    labels = [axis_label for _, axis_label in axes_order]
    before_values = [
        abs(before.get(comparison, {}).get(axis_key, float("nan")))
        for axis_key, _ in axes_order
    ]
    now_values = [
        abs(now.get(comparison, {}).get(axis_key, float("nan")))
        for axis_key, _ in axes_order
    ]
    x_values = list(range(len(labels)))
    width = 0.36
    before_color = "#4c78a8"
    now_color = "#e45756"

    plt.style.use("seaborn-v0_8-whitegrid")
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.set_title("Accuracy Error Comparison", fontweight="bold")

    finite_values = [
        value for value in before_values + now_values if math.isfinite(value)
    ]
    max_value = max(finite_values, default=1.0)
    label_offset = max(max_value * 0.015, 0.05)

    ax.bar(
        [x - width / 2 for x in x_values],
        before_values,
        width,
        label="Before",
        color=before_color,
    )
    ax.bar(
        [x + width / 2 for x in x_values],
        now_values,
        width,
        label="After",
        color=now_color,
    )

    for x, value in zip([x - width / 2 for x in x_values], before_values):
        if math.isfinite(value):
            ax.text(
                x,
                value + label_offset,
                f"{value:.2f}",
                ha="center",
                va="bottom",
                fontsize=8,
            )

    for x, value in zip([x + width / 2 for x in x_values], now_values):
        if math.isfinite(value):
            ax.text(
                x,
                value + label_offset,
                f"{value:.2f}",
                ha="center",
                va="bottom",
                fontsize=8,
            )

    ax.set_ylabel("Absolute error (deg)")
    ax.set_xticks(x_values)
    ax.set_xticklabels(labels)
    ax.legend(loc="upper right")
    ax.set_ylim(0, max_value + label_offset * 8)
    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def print_summary(title: str, before: dict, now: dict, order: list[str]) -> None:
    print()
    print(title)
    print("-" * len(title))
    for metric in order:
        before_mean = before.get(metric, {}).get("mean", float("nan"))
        now_mean = now.get(metric, {}).get("mean", float("nan"))
        change = improvement_percent(before_mean, now_mean)
        label = LATENCY_LABELS.get(metric, metric)
        print(
            f"{label:<28} before={before_mean:9.3f}  "
            f"now={now_mean:9.3f}  improvement={change:7.2f}%"
        )


def print_orientation_summary(
    title: str,
    before: dict[str, dict[str, float]],
    now: dict[str, dict[str, float]],
    order: list[str],
) -> None:
    print()
    print(title)
    print("-" * len(title))
    comparison = "Fusion-IMU" if "Fusion-IMU" in before or "Fusion-IMU" in now else order[0]
    before_row = before.get(comparison, {})
    now_row = now.get(comparison, {})
    before_text = "  ".join(
        f"{axis}={abs(before_row.get(axis, float('nan'))):8.2f}"
        for axis in ORIENTATION_TABLE_AXES
    )
    now_text = "  ".join(
        f"{axis}={abs(now_row.get(axis, float('nan'))):8.2f}"
        for axis in ORIENTATION_TABLE_AXES
    )
    print(f"{comparison:<16} before[{before_text}]  after[{now_text}]")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Generate before-vs-now accuracy and response-time comparison graphs. "
            "The time CSV can be produced by visual_response_time.py --csv. "
            "The accuracy CSV may contain error columns, or target/measured columns. "
            "Run with --demo to generate presentation graphs from estimated values."
        )
    )
    parser.add_argument("--before-time", type=Path)
    parser.add_argument("--now-time", type=Path)
    parser.add_argument("--before-accuracy", type=Path)
    parser.add_argument("--now-accuracy", type=Path)
    parser.add_argument(
        "--demo",
        action="store_true",
        help="Generate graphs from estimated demonstration values instead of CSV files.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("comparison_graphs"),
        help="Directory where PNG graphs are written.",
    )
    parser.add_argument(
        "--accuracy-unit",
        default="error",
        help="Y-axis unit for the accuracy graph, for example 'deg' or 'cm'.",
    )
    return parser.parse_args()


def demo_data():
    before_accuracy = {
        "Camera-IMU": {"roll": 6.10, "pitch": 0.25, "yaw": 169.38},
        "Fusion-Camera": {"roll": -4.97, "pitch": -0.37, "yaw": -142.77},
        "Fusion-IMU": {"roll": 1.71, "pitch": 0.89, "yaw": 26.61},
    }
    now_accuracy = {
        "Camera-IMU": {"roll": -3.74, "pitch": -1.67, "yaw": -18.79},
        "Fusion-Camera": {"roll": 2.03, "pitch": 0.78, "yaw": 9.16},
        "Fusion-IMU": {"roll": 1.13, "pitch": 0.12, "yaw": -9.63},
    }
    before_time = {
        "aruco_to_fusion_pose": {"mean": 95.0},
        "fusion_pose_to_command": {"mean": 42.0},
        "aruco_to_command": {"mean": 135.0},
        "command_to_feedback": {"mean": 165.0},
        "aruco_to_feedback": {"mean": 295.0}, 
    }
    now_time = {
        "aruco_to_fusion_pose": {"mean": 58.0},
        "fusion_pose_to_command": {"mean": 24.0},
        "aruco_to_command": {"mean": 82.0},
        "command_to_feedback": {"mean": 118.0},
        "aruco_to_feedback": {"mean": 198.0},
    }
    return before_accuracy, now_accuracy, before_time, now_time


def main() -> int:
    args = parse_args()
    if args.demo:
        before_accuracy, now_accuracy, before_time, now_time = demo_data()
    else:
        required_paths = (
            ("--before-time", args.before_time),
            ("--now-time", args.now_time),
            ("--before-accuracy", args.before_accuracy),
            ("--now-accuracy", args.now_accuracy),
        )
        for option, path in required_paths:
            if path is None:
                raise SystemExit(f"{option} is required unless --demo is used")
            if not path.is_file():
                raise SystemExit(f"Input file not found: {path}")

        before_time = summarize_time_csv(args.before_time)
        now_time = summarize_time_csv(args.now_time)
        before_accuracy = summarize_orientation_difference_csv(
            args.before_accuracy
        ) or summarize_accuracy_csv(args.before_accuracy)
        now_accuracy = summarize_orientation_difference_csv(
            args.now_accuracy
        ) or summarize_accuracy_csv(args.now_accuracy)

    latency_order = list(LATENCY_LABELS)
    orientation_bar_mode = all(
        isinstance(row, dict) and any(axis in row for axis in ORIENTATION_TABLE_AXES)
        for row in list(before_accuracy.values()) + list(now_accuracy.values())
    )
    accuracy_order = common_order(before_accuracy, now_accuracy)

    accuracy_output = args.output_dir / "accuracy_comparison.png"
    response_output = args.output_dir / "response_time_comparison.png"

    if orientation_bar_mode:
        plot_orientation_difference_bars(before_accuracy, now_accuracy, accuracy_output)
        print_orientation_summary(
            "Accuracy",
            before_accuracy,
            now_accuracy,
            common_order(before_accuracy, now_accuracy, ORIENTATION_TABLE_ORDER),
        )
    else:
        plot_comparison(
            before_accuracy,
            now_accuracy,
            accuracy_output,
            "Accuracy Comparison: Before vs Now",
            f"Mean absolute {args.accuracy_unit}",
            preferred_order=accuracy_order,
            annotation_mode="value",
        )
        print_summary("Accuracy", before_accuracy, now_accuracy, accuracy_order)
    plot_comparison(
        before_time,
        now_time,
        response_output,
        "Response Time Comparison: Before vs Now",
        "Mean latency (ms)",
        preferred_order=latency_order,
    )
    print_summary(
        "Response Time",
        before_time,
        now_time,
        common_order(before_time, now_time, latency_order),
    )
    print()
    print(f"Saved: {accuracy_output}")
    print(f"Saved: {response_output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
