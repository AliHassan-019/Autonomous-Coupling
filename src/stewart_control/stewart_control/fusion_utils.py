#!/usr/bin/env python3

"""Utilitaires pour la fusion de capteurs (angles, Kalman)."""


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

    def predict(self, x_pred):
        """
        Étape de prédiction.

        Args:
            x_pred: Prédiction de l'état (degré)
        """
        self.x = wrap_deg(float(x_pred))
        self.P = self.P + self.Q

    def update(self, z):
        """
        Étape de mise à jour avec une mesure.

        Args:
            z: Mesure (degré)
        """
        z = wrap_deg(float(z))
        y = wrap_deg(z - self.x)
        S = self.P + self.R
        if S <= 1e-12:
            return
        K = self.P / S
        self.x = wrap_deg(self.x + K * y)
        self.P = (1.0 - K) * self.P
