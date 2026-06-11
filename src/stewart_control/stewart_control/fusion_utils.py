#!/usr/bin/env python3

"""Utilitaires pour la fusion de capteurs (angles, Kalman)."""

import numpy as np


def wrap_deg(a):
    """Normalise un angle en degrés dans l'intervalle [-180, 180]."""
    return (float(a) + 180.0) % 360.0 - 180.0


class Kalman1D:
    """Filtre de Kalman 1D pour angles avec wrapping."""

    def __init__(self, q=0.02, r=1.0):
        """
        Initialise le filtre de Kalman.

        Args:
            q: Bruit de processus (covariance)
            r: Bruit de mesure (covariance)
        """
        self.x = 0.0
        self.P = 1.0
        self.Q = float(q)
        self.R = float(r)
        self.initialized = False

    def predict(self, x_pred=None):
        """
        Étape de prédiction.
        """
        self.P = self.P + self.Q
        if x_pred is not None:
            self.update(x_pred)

    def update(self, z):
        """
        Étape de mise à jour avec une mesure.

        Args:
            z: Mesure (degré)
        """
        z = wrap_deg(float(z))
        if not self.initialized:
            self.x = z
            self.initialized = True
            return

        y = wrap_deg(z - self.x)
        S = self.P + self.R
        if S <= 1e-12:
            return
        K = self.P / S
        self.x = wrap_deg(self.x + K * y)
        self.P = (1.0 - K) * self.P


class AngleRateKalman1D:
    """Kalman 1D angle-rate filter with angular wrapping and outlier gating.

    State:
        x[0] = angle in degrees
        x[1] = angular rate in degrees/second

    This gives the fusion node a motion model: real tilt can be followed through
    the estimated rate, while single-frame sensor jumps are rejected before they
    reach the platform controller.
    """

    def __init__(
        self,
        process_angle_q=0.02,
        process_rate_q=4.0,
        initial_angle_variance=2.0,
        initial_rate_variance=25.0,
        outlier_threshold_deg=8.0,
        outlier_recovery_count=3,
    ):
        self.x = np.zeros(2, dtype=float)
        self.P = np.diag(
            [float(initial_angle_variance), float(initial_rate_variance)]
        )
        self.process_angle_q = float(process_angle_q)
        self.process_rate_q = float(process_rate_q)
        self.outlier_threshold_deg = float(outlier_threshold_deg)
        self.outlier_recovery_count = int(outlier_recovery_count)
        self.initialized = False
        self.rejected_count = 0

    @property
    def angle(self):
        return wrap_deg(self.x[0])

    @property
    def rate(self):
        return float(self.x[1])

    def predict(self, dt):
        dt = max(float(dt), 1e-3)
        if not self.initialized:
            return

        F = np.array([[1.0, dt], [0.0, 1.0]], dtype=float)
        # White-acceleration style process noise. This keeps response fast
        # during real movement without letting the resting angle wander freely.
        q_angle = max(self.process_angle_q, 1e-9)
        q_rate = max(self.process_rate_q, 1e-9)
        Q = np.array(
            [
                [q_angle * dt + 0.25 * q_rate * dt**4, 0.5 * q_rate * dt**3],
                [0.5 * q_rate * dt**3, q_rate * dt**2],
            ],
            dtype=float,
        )

        self.x = F @ self.x
        self.x[0] = wrap_deg(self.x[0])
        self.P = F @ self.P @ F.T + Q

    def update(self, measurement_deg, measurement_variance, allow_outlier_recovery=True):
        measurement = wrap_deg(measurement_deg)
        measurement_variance = max(float(measurement_variance), 1e-6)

        if not self.initialized:
            self.x[0] = measurement
            self.x[1] = 0.0
            self.initialized = True
            self.rejected_count = 0
            return True

        innovation = wrap_deg(measurement - self.x[0])
        if abs(innovation) > self.outlier_threshold_deg:
            self.rejected_count += 1
            if (
                not allow_outlier_recovery
                or self.rejected_count < self.outlier_recovery_count
            ):
                return False
        else:
            self.rejected_count = 0

        H = np.array([[1.0, 0.0]], dtype=float)
        S = float((H @ self.P @ H.T)[0, 0] + measurement_variance)
        if S <= 1e-12:
            return False

        K = (self.P @ H.T)[:, 0] / S
        self.x = self.x + K * innovation
        self.x[0] = wrap_deg(self.x[0])
        self.P = (np.eye(2) - np.outer(K, H[0])) @ self.P
        self.rejected_count = 0
        return True


class OrientationKalmanFilter:
    """Three-axis RPY Kalman filter using angle + angular-rate states."""

    AXES = ("roll", "pitch", "yaw")

    def __init__(self, axis_configs):
        self.filters = [
            AngleRateKalman1D(
                process_angle_q=axis_configs[axis]["process_angle_q"],
                process_rate_q=axis_configs[axis]["process_rate_q"],
                outlier_threshold_deg=axis_configs[axis]["outlier_threshold_deg"],
                outlier_recovery_count=axis_configs[axis]["outlier_recovery_count"],
            )
            for axis in self.AXES
        ]

    def predict(self, dt):
        for angle_filter in self.filters:
            angle_filter.predict(dt)

    def update(self, measurements, variances, allow_outlier_recovery=True):
        accepted = []
        for angle_filter, measurement, variance in zip(
            self.filters, measurements, variances
        ):
            accepted.append(
                angle_filter.update(
                    measurement,
                    variance,
                    allow_outlier_recovery=allow_outlier_recovery,
                )
            )
        return accepted

    @property
    def orientation(self):
        return [angle_filter.angle for angle_filter in self.filters]

    @property
    def rates(self):
        return [angle_filter.rate for angle_filter in self.filters]
