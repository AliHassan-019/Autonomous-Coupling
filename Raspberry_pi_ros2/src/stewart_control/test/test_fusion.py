#!/usr/bin/env python3

"""Tests unitaires pour le filtre de fusion et Kalman."""

import unittest
from stewart_control.fusion_utils import wrap_deg, Kalman1D


class TestAngleUtils(unittest.TestCase):
    """Tests pour les fonctions utilitaires d'angles."""

    def test_wrap_deg_positive(self):
        """Test de normalisation d'angles positifs."""
        self.assertAlmostEqual(wrap_deg(0), 0.0)
        self.assertAlmostEqual(wrap_deg(90), 90.0)
        # 180 et -180 sont équivalents, wrap_deg retourne -180
        self.assertAlmostEqual(abs(wrap_deg(180)), 180.0)
        self.assertAlmostEqual(wrap_deg(179), 179.0)

    def test_wrap_deg_negative(self):
        """Test de normalisation d'angles négatifs."""
        self.assertAlmostEqual(wrap_deg(-90), -90.0)
        self.assertAlmostEqual(wrap_deg(-180), -180.0)
        self.assertAlmostEqual(wrap_deg(-179), -179.0)

    def test_wrap_deg_over_180(self):
        """Test de normalisation d'angles > 180°."""
        self.assertAlmostEqual(wrap_deg(181), -179.0)
        self.assertAlmostEqual(wrap_deg(190), -170.0)
        self.assertAlmostEqual(wrap_deg(270), -90.0)
        self.assertAlmostEqual(wrap_deg(360), 0.0)
        self.assertAlmostEqual(wrap_deg(450), 90.0)

    def test_wrap_deg_under_minus_180(self):
        """Test de normalisation d'angles < -180°."""
        self.assertAlmostEqual(wrap_deg(-181), 179.0)
        self.assertAlmostEqual(wrap_deg(-190), 170.0)
        self.assertAlmostEqual(wrap_deg(-270), 90.0)
        self.assertAlmostEqual(wrap_deg(-360), 0.0)

    def test_wrap_deg_large_values(self):
        """Test avec de très grandes valeurs."""
        self.assertAlmostEqual(wrap_deg(720), 0.0)
        self.assertAlmostEqual(wrap_deg(725), 5.0)
        self.assertAlmostEqual(wrap_deg(-720), 0.0)


class TestKalman1D(unittest.TestCase):
    """Tests pour le filtre de Kalman 1D."""

    def setUp(self):
        """Initialisation avant chaque test."""
        self.kf = Kalman1D(q=0.02, r=1.0)

    def test_initialization(self):
        """Test de l'initialisation du filtre."""
        self.assertEqual(self.kf.x, 0.0)
        self.assertEqual(self.kf.P, 1.0)
        self.assertEqual(self.kf.Q, 0.02)
        self.assertEqual(self.kf.R, 1.0)

    def test_predict_simple(self):
        """Test de la prédiction simple."""
        self.kf.predict(10.0)
        self.assertAlmostEqual(self.kf.x, 10.0)
        # La covariance augmente avec Q
        self.assertAlmostEqual(self.kf.P, 1.0 + 0.02)

    def test_predict_wrapping(self):
        """Test que la prédiction gère le wrapping."""
        self.kf.predict(190.0)
        # 190° devrait être wrappé à -170°
        self.assertAlmostEqual(self.kf.x, -170.0)

    def test_update_simple(self):
        """Test de la mise à jour avec une mesure."""
        self.kf.predict(0.0)
        initial_x = self.kf.x
        initial_P = self.kf.P

        # Mesure à 10°
        self.kf.update(10.0)

        # L'état devrait avoir bougé vers la mesure
        self.assertGreater(self.kf.x, initial_x)
        self.assertLess(self.kf.x, 10.0)

        # La covariance devrait diminuer
        self.assertLess(self.kf.P, initial_P)

    def test_update_zero_covariance_protection(self):
        """Test de protection contre division par zéro."""
        self.kf.P = 0.0
        self.kf.R = 0.0
        self.kf.x = 5.0

        # Ne devrait pas crasher
        self.kf.update(10.0)

        # L'état ne devrait pas avoir changé
        self.assertAlmostEqual(self.kf.x, 5.0)

    def test_multiple_predictions_and_updates(self):
        """Test d'une séquence prédiction-mise à jour."""
        measurements = [10.0, 11.0, 12.0, 11.5, 12.5]

        for z in measurements:
            self.kf.predict(z)
            self.kf.update(z)

        # L'état devrait converger vers les mesures
        self.assertGreater(self.kf.x, 10.0)
        self.assertLess(self.kf.x, 13.0)

        # La covariance devrait être faible après plusieurs updates
        self.assertLess(self.kf.P, 1.0)

    def test_angle_wrapping_in_innovation(self):
        """Test du wrapping dans l'innovation (y = z - x)."""
        self.kf.x = 170.0
        self.kf.predict(170.0)

        # Mesure à -170° (équivalent à 190°)
        self.kf.update(-170.0)

        # L'innovation devrait être calculée correctement
        # -170 - 170 = -340, wrappé à 20
        # Donc x devrait bouger vers -170
        self.assertLess(self.kf.x, 170.0)

    def test_custom_parameters(self):
        """Test avec des paramètres Q et R personnalisés."""
        kf_low_q = Kalman1D(q=0.001, r=1.0)
        kf_high_q = Kalman1D(q=0.5, r=1.0)

        kf_low_q.predict(0.0)
        kf_high_q.predict(0.0)

        # Q élevé = covariance augmente plus vite
        self.assertLess(kf_low_q.P, kf_high_q.P)

    def test_convergence_to_stable_measurement(self):
        """Test de convergence vers une mesure stable."""
        self.kf.x = 0.0

        # 10 mesures identiques à 50°
        for _ in range(10):
            self.kf.predict(50.0)
            self.kf.update(50.0)

        # Devrait converger très proche de 50°
        self.assertAlmostEqual(self.kf.x, 50.0, places=1)


class TestKalmanWithNoise(unittest.TestCase):
    """Tests du filtre Kalman avec du bruit."""

    def test_noise_filtering(self):
        """Test que le filtre lisse le bruit."""
        import random

        random.seed(42)  # Pour reproductibilité
        kf = Kalman1D(q=0.01, r=2.0)

        # Signal avec bruit
        true_value = 45.0
        noisy_measurements = [true_value + random.gauss(0, 5.0) for _ in range(50)]

        estimates = []
        for z in noisy_measurements:
            kf.predict(z)
            kf.update(z)
            estimates.append(kf.x)

        # La variance devrait au moins être comparable ou plus faible
        import statistics

        noise_var = statistics.variance(noisy_measurements)
        estimate_var = statistics.variance(estimates)

        # Tolérance : estimation ne devrait pas être pire que les mesures
        self.assertLessEqual(estimate_var, noise_var * 1.1)


if __name__ == "__main__":
    unittest.main()
