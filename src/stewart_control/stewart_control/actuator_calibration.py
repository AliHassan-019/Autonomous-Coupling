#!/usr/bin/env python3

"""Calibration helpers for per-motor actuator limits."""

import numpy as np


class ActuatorCalibration:
    """Centralizes per-motor soft limits and synchronized stroke mapping."""

    MOTOR_COUNT = 6

    def __init__(
        self,
        motor_min_cm,
        motor_max_cm,
        logical_min_cm=0.0,
        logical_max_cm=10.0,
    ):
        self.motor_min_cm = self._as_vector(motor_min_cm, "motor_min_cm")
        self.motor_max_cm = self._as_vector(motor_max_cm, "motor_max_cm")
        self.logical_min_cm = float(logical_min_cm)
        self.logical_max_cm = float(logical_max_cm)

        if np.any(self.motor_max_cm <= self.motor_min_cm):
            raise ValueError("Every motor max must be strictly greater than its min.")

        if self.logical_max_cm <= self.logical_min_cm:
            raise ValueError("logical_max_cm must be strictly greater than logical_min_cm.")

    @classmethod
    def from_config(cls, actuators_cfg):
        """Build a calibration helper from the actuator config block."""

        return cls(
            motor_min_cm=actuators_cfg["motor_min_cm"],
            motor_max_cm=actuators_cfg["motor_max_cm"],
            logical_min_cm=actuators_cfg.get("logical_min_cm", 0.0),
            logical_max_cm=actuators_cfg.get("logical_max_cm", 10.0),
        )

    def _as_vector(self, values, field_name):
        vector = np.array(values, dtype=float)
        if vector.shape != (self.MOTOR_COUNT,):
            raise ValueError(f"{field_name} must contain exactly {self.MOTOR_COUNT} values.")
        return vector

    @property
    def common_span_cm(self):
        return self.logical_max_cm - self.logical_min_cm

    @property
    def motor_span_cm(self):
        return self.motor_max_cm - self.motor_min_cm

    def within_logical_range(self, logical_targets_cm):
        logical_targets = self._as_vector(logical_targets_cm, "logical_targets_cm")
        return bool(
            np.all(logical_targets >= self.logical_min_cm)
            and np.all(logical_targets <= self.logical_max_cm)
        )

    def within_limits(self, targets_cm):
        targets = self._as_vector(targets_cm, "targets_cm")
        return bool(
            np.all(targets >= self.motor_min_cm) and np.all(targets <= self.motor_max_cm)
        )

    def clamp_logical_targets(self, logical_targets_cm):
        logical_targets = self._as_vector(logical_targets_cm, "logical_targets_cm")
        return np.clip(logical_targets, self.logical_min_cm, self.logical_max_cm)

    def clamp_targets(self, targets_cm):
        targets = self._as_vector(targets_cm, "targets_cm")
        return np.clip(targets, self.motor_min_cm, self.motor_max_cm)

    def logical_to_physical(self, logical_targets_cm):
        """Map common logical stroke values to calibrated per-motor positions."""

        logical_targets = self._as_vector(logical_targets_cm, "logical_targets_cm")
        ratio = (logical_targets - self.logical_min_cm) / self.common_span_cm
        ratio = np.clip(ratio, 0.0, 1.0)
        return self.motor_min_cm + ratio * self.motor_span_cm

    def logical_to_relative_targets(self, logical_targets_cm, reference_logical_cm=None):
        """Map logical targets into session-relative motor positions."""

        if reference_logical_cm is None:
            reference_logical_cm = [self.logical_min_cm] * self.MOTOR_COUNT
        return self.logical_to_physical(logical_targets_cm) - self.logical_to_physical(
            reference_logical_cm
        )

    def relative_min_cm(self, reference_logical_cm=None):
        if reference_logical_cm is None:
            reference_logical_cm = [self.logical_min_cm] * self.MOTOR_COUNT
        return self.motor_min_cm - self.logical_to_physical(reference_logical_cm)

    def relative_max_cm(self, reference_logical_cm=None):
        if reference_logical_cm is None:
            reference_logical_cm = [self.logical_min_cm] * self.MOTOR_COUNT
        return self.motor_max_cm - self.logical_to_physical(reference_logical_cm)

    def within_relative_limits(self, targets_cm, reference_logical_cm=None):
        targets = self._as_vector(targets_cm, "targets_cm")
        return bool(
            np.all(targets >= self.relative_min_cm(reference_logical_cm))
            and np.all(targets <= self.relative_max_cm(reference_logical_cm))
        )

    def relative_violation_message(self, targets_cm, reference_logical_cm=None):
        targets = self._as_vector(targets_cm, "targets_cm")
        lower_bounds = self.relative_min_cm(reference_logical_cm)
        upper_bounds = self.relative_max_cm(reference_logical_cm)
        violations = []
        for idx, value in enumerate(targets):
            lower = lower_bounds[idx]
            upper = upper_bounds[idx]
            if value < lower or value > upper:
                violations.append(
                    f"M{idx + 1}={value:.2f} cm outside [{lower:.2f}, {upper:.2f}]"
                )
        return "; ".join(violations)

    def synchronized_stroke_to_physical(self, stroke_cm):
        """Map one shared stroke value to all motors while preserving sync."""

        return self.logical_to_physical([stroke_cm] * self.MOTOR_COUNT)

    def logical_violation_message(self, logical_targets_cm):
        logical_targets = self._as_vector(logical_targets_cm, "logical_targets_cm")
        violations = []
        for idx, value in enumerate(logical_targets):
            if value < self.logical_min_cm or value > self.logical_max_cm:
                violations.append(
                    f"M{idx + 1} logical={value:.2f} cm outside "
                    f"[{self.logical_min_cm:.2f}, {self.logical_max_cm:.2f}]"
                )
        return "; ".join(violations)

    def violation_message(self, targets_cm):
        targets = self._as_vector(targets_cm, "targets_cm")
        violations = []
        for idx, value in enumerate(targets):
            lower = self.motor_min_cm[idx]
            upper = self.motor_max_cm[idx]
            if value < lower or value > upper:
                violations.append(
                    f"M{idx + 1}={value:.2f} cm outside [{lower:.2f}, {upper:.2f}]"
                )
        return "; ".join(violations)
