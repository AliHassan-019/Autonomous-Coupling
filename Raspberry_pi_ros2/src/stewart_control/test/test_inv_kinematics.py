#!/usr/bin/env python3

"""Tests unitaires pour le module de cinématique inverse."""

import unittest
import numpy as np
from stewart_control.inv_kinematics import StewartPlatform


class TestStewartPlatform(unittest.TestCase):
    """Tests pour la classe StewartPlatform."""

    def setUp(self):
        """Initialisation avant chaque test."""
        # Paramètres réels du projet
        self.platform = StewartPlatform(
            radious_base=0.075,
            radious_platform=0.04,
            gamma_base=11.3,
            gamma_platform=17.3,
        )

    def test_initialization(self):
        """Test de l'initialisation de la plateforme."""
        self.assertIsNotNone(self.platform)
        self.assertEqual(self.platform.rb, 0.075)
        self.assertEqual(self.platform.rp, 0.04)
        np.testing.assert_array_equal(self.platform.home_pos, np.array([0, 0, 0.185]))

    def test_frame_generation(self):
        """Test de la génération des frames de base et plateforme."""
        P, B = self.platform.frame()

        # Vérifier les dimensions
        self.assertEqual(P.shape, (3, 6))
        self.assertEqual(B.shape, (3, 6))

        # Vérifier que les points sont sur un cercle
        for i in range(6):
            radius_p = np.sqrt(P[0, i] ** 2 + P[1, i] ** 2)
            radius_b = np.sqrt(B[0, i] ** 2 + B[1, i] ** 2)
            self.assertAlmostEqual(radius_p, self.platform.rp, places=10)
            self.assertAlmostEqual(radius_b, self.platform.rb, places=10)

        # Vérifier que z=0 pour tous les points
        np.testing.assert_array_almost_equal(P[2, :], np.zeros(6))
        np.testing.assert_array_almost_equal(B[2, :], np.zeros(6))

    def test_solve_home_position(self):
        """Test de la cinématique inverse à la position home."""
        trans = np.zeros(3)
        rotation = np.zeros(3)

        lengths = self.platform.solve(trans, rotation)

        # 6 longueurs retournées
        self.assertEqual(lengths.shape, (6,))

        # Toutes les longueurs devraient être égales (symétrie)
        for length in lengths:
            self.assertAlmostEqual(length, lengths[0], places=6)

        # Longueurs dans une plage raisonnable (> hauteur Z car inclinés)
        self.assertGreater(lengths[0], 0.185)
        self.assertLess(lengths[0], 0.25)

    def test_solve_translation_z(self):
        """Test d'une translation pure en z."""
        trans = np.array([0, 0, 0.02])  # 2cm vers le haut
        rotation = np.zeros(3)

        lengths = self.platform.solve(trans, rotation)

        # 6 longueurs retournées
        self.assertEqual(lengths.shape, (6,))

        # Toutes les longueurs augmentent de la même façon (symétrie)
        for length in lengths:
            self.assertAlmostEqual(length, lengths[0], places=6)

        # Les longueurs augmentent par rapport à la position home
        lengths_home = self.platform.solve(np.zeros(3), np.zeros(3))
        for i in range(6):
            self.assertGreater(lengths[i], lengths_home[i])

    def test_solve_rotation_roll(self):
        """Test d'une rotation pure en roulis."""
        trans = np.zeros(3)
        rotation = np.array([5.0, 0, 0])  # 5° de roulis

        lengths = self.platform.solve(trans, rotation)

        # 6 longueurs retournées
        self.assertEqual(lengths.shape, (6,))

        # Les longueurs ne devraient pas toutes être identiques
        self.assertGreater(np.std(lengths), 0.001)

    def test_solve_combined_motion(self):
        """Test d'un mouvement combiné translation + rotation."""
        trans = np.array([0.01, 0.01, 0.02])
        rotation = np.array([2.0, 3.0, 1.0])

        lengths = self.platform.solve(trans, rotation)

        # Vérifications de base
        self.assertEqual(lengths.shape, (6,))
        # Toutes les longueurs doivent être positives
        self.assertTrue(np.all(lengths > 0))
        # Longueurs raisonnables (entre 10cm et 30cm)
        self.assertTrue(np.all(lengths > 0.10))
        self.assertTrue(np.all(lengths < 0.30))

    def test_rotation_matrices(self):
        """Test des matrices de rotation."""
        angle = 90.0

        # Test rotX
        Rx = self.platform.rotX(angle)
        self.assertEqual(Rx.shape, (3, 3))
        # Déterminant = 1 pour une rotation
        self.assertAlmostEqual(np.linalg.det(Rx), 1.0, places=10)

        # Test rotY
        Ry = self.platform.rotY(angle)
        self.assertEqual(Ry.shape, (3, 3))
        self.assertAlmostEqual(np.linalg.det(Ry), 1.0, places=10)

        # Test rotZ
        Rz = self.platform.rotZ(angle)
        self.assertEqual(Rz.shape, (3, 3))
        self.assertAlmostEqual(np.linalg.det(Rz), 1.0, places=10)

    def test_solve_large_displacement_warning(self):
        """Test que les grands déplacements donnent des résultats cohérents."""
        trans = np.array([0, 0, 0.1])  # 10cm (limite max)
        rotation = np.zeros(3)

        lengths = self.platform.solve(trans, rotation)

        # Vérifier que les longueurs restent dans une plage raisonnable
        self.assertTrue(np.all(lengths > 0))
        self.assertTrue(np.all(lengths < 0.40))


if __name__ == "__main__":
    unittest.main()
