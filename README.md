# Projet d'attelage autonome

[![CI - Stewart Control](https://github.com/ABMI-software/Demonstrateur_REM/actions/workflows/ci.yml/badge.svg)](https://github.com/ABMI-software/Demonstrateur_REM/actions/workflows/ci.yml)

Ce projet a pour objectif de développer un système d'attelage automatique entre un tracteur et une remorque grâce au pilotage d'une plateforme de mouvement à six degrés de liberté (6 DOF) — Stewart platform.

Le système combine plusieurs capteurs pour estimer précisément la position relative des deux éléments et ajuste en temps réel les mouvements de la plateforme afin d'aligner et d'atteler les véhicules de façon autonome et précise.

Le traitement des données et la logique de contrôle sont répartis entre une Raspberry Pi (calcul et traitement des capteurs) et une carte Arduino (commande des moteurs).

## 📂 Structure du dépôt

| Dossier               | Contenu                                                              |
|------------------------|----------------------------------------------------------------------|
| `Raspberry_pi_ros2/`  | Workspace ROS 2 colcon (nœuds Python, tests, CI/CD, Arduino)        |
| `Raspberry_pi/`       | Scripts Raspberry Pi hors ROS                                        |
| `Consignes_Moteurs/`  | Programmes de commande moteur (ancienne version)                     |
| `imu/`                | Scripts d'acquisition IMU standalone                                 |
| `consignes_pi.ino`    | Sketch Arduino de consignes                                         |

## 🛠️ Matériel utilisé

| Matériel                 | Rôle                                                               |
|--------------------------|---------------------------------------------------------------------|
| **Raspberry Pi 5**       | Exécute ROS 2 Jazzy, traite vision et IMU, calcule consignes.      |
| **Arduino Mega**         | Pilote les 6 moteurs (PWM, direction, vitesse).                    |
| **6 moteurs JGA25370**   | Actionnement de la plateforme Stewart.                              |
| **6 drivers DRV8871**    | Contrôle des moteurs DC.                                            |
| **Caméra + ArUco**       | Détection de position et orientation (DICT_4X4_50, ID 34/28).      |
| **2× IMU MPU-9250**      | Mesure orientation relative (I2C 0x68 / 0x69).                     |
| **Écran tactile**        | Interface utilisateur pour contrôle et monitoring.                  |
| **Plateforme mécanique** | Plateforme Stewart 6-DOF (6 vérins linéaires) — **montée et testée** ✅ |
| **Boîtier AttelCore**    | Boîtier de mesure embarqué (Raspberry Pi + écran tactile + arrêt d'urgence) — **monté et testé** ✅ |

## Rôle de chaque carte

**Raspberry Pi 5** :
- Gère les capteurs haut niveau (caméra, IMU)
- Calcule les consignes de mouvement (cinématique inverse)
- Fusion de données capteurs via filtre de Kalman
- Envoie les consignes vers l'Arduino via UART

**Arduino Mega** :
- Reçoit les consignes via UART3
- Pilote les 6 moteurs via les drivers DRV8871
- Gère le contrôle bas niveau (PWM, direction, vitesse)

## 🧩 Architecture générale

![Architecture](docs/architecture.png)

Le projet s'articule autour de **ROS 2 Jazzy** (sur Raspberry Pi) et d'un **Arduino** pour le contrôle bas niveau.

### 🔹 Flux de données

1. `Interface_node` est le **point d'entrée** :
   - Un clic sur **Commencer** démarre automatiquement :
     - `aruco_node` (vision)
     - `IMU_node` (orientation)
     - `fusion_node` (fusion Kalman)
     - `stewart_node` (calcul longueurs vérins)
2. Les données de **position** (`/aruco_positions`) et d'**orientation** (`/IMU_error`) sont reçues en temps réel.
3. `fusion_node` fusionne les mesures via un **filtre de Kalman 1D** (roll, pitch, yaw).
4. `stewart_node` calcule les longueurs et publie `/longueurs_verins`.
5. L'interface affiche **toutes les données en live**.
6. Le bouton **Arrêter** stoppe tout sauf l'interface.

### 🔹 Nœuds ROS 2

| Nœud                 | Rôle                                                                        |
|----------------------|-----------------------------------------------------------------------------|
| `Interface_node`     | Interface graphique (PySide6), gestion du système et visualisation live.    |
| `aruco_node`         | Détection des marqueurs ArUco, publication `/aruco_positions`.              |
| `IMU_node`           | Lecture 2× MPU-9250 et publication des données IMU `/IMU_error`.            |
| `fusion_node`        | Fusion Kalman des mesures ArUco + IMU, publication `/fused_orientation`.    |
| `stewart_node`       | Calcul des longueurs vérins (cinématique inverse 6-DOF).                    |
| `Programme_moteurs`  | Code Arduino : exécution des consignes moteurs reçues via UART3.            |

### 🔹 Topics ROS 2

| Topic                 | Publié par       | Contenu                                       |
|-----------------------|------------------|-----------------------------------------------|
| `/aruco_positions`    | `aruco_node`     | Position (x, y, z).                           |
| `/IMU_error`          | `IMU_node`       | Orientation IMU (roll, yaw, pitch).            |
| `/fused_orientation`  | `fusion_node`    | Orientation fusionnée (filtre de Kalman).      |
| `/longueurs_verins`   | `stewart_node`   | Longueurs calculées pour chaque vérin.         |

<img width="1619" height="964" alt="Architecture du système" src="https://github.com/user-attachments/assets/4354006b-75d7-4ca8-85bc-1d1d9ccaf691" />

---

## 🚀 Utilisation

```bash
# 1 — Compiler le workspace ROS 2
cd Raspberry_pi_ros2
colcon build --symlink-install
source install/setup.bash

# 2 — Lancer l'interface (point d'entrée)
ros2 run stewart_control interface_node

# 3 — Ou lancer tous les nœuds via le launch file
ros2 launch stewart_control stewart_ordered_launch.py
```

Dans l'interface :
- **Commencer** → lance `aruco_node`, `IMU_node`, `fusion_node`, `stewart_node`
- **Arrêter** → stoppe les nœuds (équivalent Ctrl+C), l'interface reste ouverte

---

## 🧪 Tests

Le projet inclut **23 tests unitaires** couvrant les modules critiques :

| Fichier                     | Tests | Couverture                                          |
|-----------------------------|-------|-----------------------------------------------------|
| `test_inv_kinematics.py`    | 8     | Position home, limites, symétrie, bras, singularités |
| `test_fusion.py`            | 15    | Kalman predict/update, wrap_deg, convergence, bruit  |

```bash
# Installer pytest
pip install --user pytest

# Lancer tous les tests
cd Raspberry_pi_ros2
python -m pytest src/stewart_control/test/test_inv_kinematics.py \
                 src/stewart_control/test/test_fusion.py -v
```

---

## ✅ Qualité du code

Le projet utilise les outils suivants pour garantir la qualité :

| Outil          | Rôle                                               |
|----------------|-----------------------------------------------------|
| **Black**      | Formatage automatique Python (ligne max 88 car.)    |
| **flake8**     | Linting Python (PEP 8, complexité, erreurs)         |
| **pre-commit** | Exécution automatique avant chaque commit            |
| **pytest**     | 23 tests unitaires (cinématique + fusion Kalman)     |

### Installation des outils

```bash
pip install --user pre-commit black flake8 pytest
cd Raspberry_pi_ros2
pre-commit install
pre-commit run --all-files   # vérification initiale
```

### Utilisation quotidienne

Les hooks s'exécutent automatiquement à chaque `git commit`. Pour lancer manuellement :

```bash
pre-commit run --all-files
```

---

## 🔄 CI/CD — Intégration Continue

Le projet dispose d'un **pipeline GitHub Actions** qui s'exécute automatiquement à chaque push et pull request.

### Jobs du pipeline

| Job            | Environnement              | Actions                                          |
|----------------|----------------------------|--------------------------------------------------|
| **Lint**       | Ubuntu 24.04               | Black (vérification) + flake8                    |
| **Unit Tests** | Ubuntu 24.04               | pytest — 23 tests (cinématique + fusion)         |
| **Build ROS2** | `osrf/ros:jazzy-desktop`   | `colcon build` + `colcon test`                   |
| **Arduino**    | Ubuntu 24.04               | `arduino-cli compile` (FQBN `arduino:avr:mega`) |

Voir le fichier [`Raspberry_pi_ros2/.github/workflows/ci.yml`](Raspberry_pi_ros2/.github/workflows/ci.yml) pour la configuration complète.

---

## 🌿 Branches Git

| Branche            | Rôle                                                    |
|--------------------|---------------------------------------------------------|
| `main`             | Version stable et validée                               |
| `develop-jbantu`   | Branche de développement (fonctionnalités en cours)     |
| Branches `feature/`| Branches de fonctionnalités individuelles               |

### Workflow de contribution

1. Créer une branche `feature/...` ou `bugfix/...` depuis `main`.
2. Développer, formater (`black`), tester (`pytest`).
3. Pousser et créer une **Pull Request**.
4. Le pipeline CI valide automatiquement (lint, tests, build, Arduino).
5. Merge après revue.

---

## 📌 État actuel du projet

### ✅ Matériel monté et testé (février 2026)

- **Plateforme Stewart** — Plateforme mécanique 6-DOF assemblée avec 6 vérins linéaires (JGA25370 + DRV8871). Montage validé, tests de mouvement effectués.
- **Boîtier de mesure AttelCore** — Boîtier embarqué intégrant la Raspberry Pi 5, un écran tactile, un bouton d'arrêt d'urgence et les connectiques (Ethernet, USB, alimentation). Monté et opérationnel.

### ✅ Fonctionnalités logicielles opérationnelles

- **Architecture ROS 2** — Nœuds `aruco_node`, `IMU_node`, `fusion_node`, `stewart_node` et `Interface_node` fonctionnent ensemble. Communication stable entre les nœuds.
- **Interface graphique (PySide6)** — Lancement/arrêt du système, affichage temps réel des positions, orientations et longueurs de vérins.
- **Cinématique inverse** — Calcul des 6 longueurs de vérins en fonction de la position et orientation 6-DOF. Paramètres géométriques : rb=0.075m, rp=0.04m, home=[0, 0, 0.185m].
- **Fusion de données (Kalman)** — Filtre de Kalman 1D fusionnant les mesures ArUco et IMU (roll, pitch, yaw) avec gestion du wrapping angulaire.
- **Vision ArUco** — Détection des marqueurs (DICT_4X4_50 : ID 34 fixe, ID 28 mobile) et estimation de position.
- **Acquisition IMU** — Lecture de 2× MPU-9250 (I2C 0x68/0x69) et publication sur `/IMU_error`.
- **Communication Raspberry Pi ↔ Arduino** — Liaison UART fonctionnelle, transmission fiable des consignes.
- **Commande moteur (Arduino)** — Contrôle 6 moteurs DC via DRV8871, réception et exécution des consignes.

### ✅ Bonnes pratiques de développement (février 2026)

- **Pipeline CI/CD** — 4 jobs GitHub Actions (lint, tests, build ROS2, Arduino) — tous au vert ✅
- **23 tests unitaires** — Cinématique inverse (8 tests) + Fusion Kalman (15 tests) avec pytest.
- **Formatage automatique** — Black + flake8 + pre-commit hooks.
- **Versionnement Git** — Stratégie main/develop avec pull requests et revue de code.
- **Code nettoyé** — Variables renommées, imports inutilisés supprimés, exceptions typées, `fusion_utils.py` extrait pour testabilité.

### ⚠️ Travail restant

- Calibration de la caméra et des IMU (les valeurs actuelles ne sont pas fiables à 100%).
- Externalisation des constantes dans des fichiers de configuration YAML.
- Documentation d'architecture complète (diagramme de nœuds/topics).
