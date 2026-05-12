# Manuel d'utilisation — Interface graphique Stewart Platform

> **ABMI Groupe** · Stewart Platform — Contrôle & Monitoring
> Version 0.1.0 · Février 2026

---

## Table des matières

1. [Prérequis](#1-prérequis)
2. [Lancement](#2-lancement)
3. [Vue d'ensemble de l'interface](#3-vue-densemble-de-linterface)
4. [Colonne gauche](#4-colonne-gauche)
   - 4.1 [En-tête](#41-en-tête)
   - 4.2 [Contrôle manuel](#42-contrôle-manuel)
   - 4.3 [Actions](#43-actions)
   - 4.4 [Caméra ArUco](#44-caméra-aruco)
5. [Colonne droite](#5-colonne-droite)
   - 5.1 [Carte ARUCO](#51-carte-aruco)
   - 5.2 [Carte IMU](#52-carte-imu)
   - 5.3 [Carte FUSION](#53-carte-fusion)
   - 5.4 [Carte MOTEURS](#54-carte-moteurs)
   - 5.5 [Graphes moteurs M1–M6](#55-graphes-moteurs-m1m6)
6. [Indicateurs LED](#6-indicateurs-led)
7. [Mode Démo (sans ROS2)](#7-mode-démo-sans-ros2)
8. [Raccourcis et astuces](#8-raccourcis-et-astuces)
9. [Dépannage](#9-dépannage)

---

## 1. Prérequis

| Composant | Version minimale |
|---|---|
| ROS2 | Jazzy Jalisco |
| Python | 3.12 |
| PySide6 | 6.8+ |
| matplotlib | 3.8+ |
| numpy | 1.26+ |

**Installation rapide des dépendances Python (système) :**

```bash
/usr/bin/python3 -m pip install --user --break-system-packages PySide6 matplotlib numpy
```

---

## 2. Lancement

### Mode normal (avec ROS2)

```bash
cd ~/ros2_MP_ws
source install/setup.bash
ros2 run stewart_control interface_node
```

### Mode démo (sans ROS2)

```bash
cd ~/ros2_MP_ws
./run_demo.sh
```

Le mode démo simule des données capteurs à 20 Hz pour valider visuellement l'interface sans matériel connecté.

---

## 3. Vue d'ensemble de l'interface

L'interface est organisée en **deux colonnes** sur fond sombre :

```
┌────────────────────┬──────────────────────────────────────────────┐
│   COLONNE GAUCHE   │              COLONNE DROITE                  │
│     (30%)          │                (70%)                         │
│                    │                                              │
│ ┌────────────────┐ │ ┌──────────┬──────────┬──────────┐          │
│ │  Logo + Titre  │ │ │  ARUCO   │   IMU    │  FUSION  │          │
│ └────────────────┘ │ └──────────┴──────────┴──────────┘          │
│ ┌────────────────┐ │ ┌────────────────────────────────┐          │
│ │ CONTRÔLE       │ │ │         MOTEURS                │          │
│ │   MANUEL       │ │ └────────────────────────────────┘          │
│ └────────────────┘ │ ┌──────────┬──────────┬──────────┐          │
│ ┌────────────────┐ │ │   M1     │   M2     │   M3     │          │
│ │   ACTIONS      │ │ ├──────────┼──────────┼──────────┤          │
│ └────────────────┘ │ │   M4     │   M5     │   M6     │          │
│ ┌────────────────┐ │ └──────────┴──────────┴──────────┘          │
│ │ CAMÉRA ARUCO   │ │                                              │
│ │                │ │                                              │
│ └────────────────┘ │                                              │
└────────────────────┴──────────────────────────────────────────────┘
```

---

## 4. Colonne gauche

### 4.1 En-tête

Affiche le **logo ABMI**, le titre **« Stewart Platform »** et le sous-titre **« ABMI Groupe — Contrôle & Monitoring »**.

### 4.2 Contrôle manuel

Permet de commander la plateforme en **position** et **orientation** :

| Champ | Plage | Unité | Pas |
|---|---|---|---|
| X | −1.000 à +1.000 | cm | 0.01 |
| Y | −1.000 à +1.000 | cm | 0.01 |
| Z | −1.000 à +1.000 | cm | 0.01 |
| Roll | −180.0 à +180.0 | ° (degrés) | 1.0 |
| Pitch | −180.0 à +180.0 | ° (degrés) | 1.0 |
| Yaw | −180.0 à +180.0 | ° (degrés) | 1.0 |

**Utilisation :**

1. Réglez les valeurs souhaitées avec les spinboxes (clic sur les flèches ou saisie directe).
2. Cliquez **▶ Appliquer** pour envoyer la consigne.

> ⚠️ Le bouton **Appliquer** ne fonctionne que si le mode **Manuel** est actif (voir section suivante).

Les valeurs sont publiées sur les topics ROS2 :
- `manual_position` — X, Y, Z
- `manual_orientation` — Roll, Pitch, Yaw

### 4.3 Actions

Cinq boutons permettent de démarrer/arrêter les différents modes de la plateforme :

| Bouton | Couleur | Description | Launch file |
|---|---|---|---|
| **▸ Acquisition** | Bleu | Démarre l'acquisition des capteurs (ArUco, IMU) + fusion | `stewart_ordered_launch.py` |
| **▸ Automatique** | Vert | Mode asservissement automatique | `stewart.launch.py` |
| **▸ Manuel** | Violet | Active le contrôle manuel (spinboxes) | `manual_launch.py` |
| **▸ État initial** | Gris | Ramène la plateforme en position de repos | `launch_posHome.py` |
| **■ ARRÊTER** | Rouge | Arrête **tous** les processus en cours | — |

**Comportement :**

- Un seul mode peut être actif à la fois. Le **mode courant** est affiché en vert (ex. : `● ACQUISITION`).
- Cliquer sur un bouton déjà actif n'a aucun effet (protection anti-doublon).
- Le bouton **ARRÊTER** envoie un `SIGINT` à tous les processus puis force un `kill` après 5 secondes si nécessaire.

### 4.4 Caméra ArUco

Affiche le **flux vidéo en temps réel** de la caméra avec la détection des marqueurs ArUco superposée.

- **Source** : topic ROS2 `camera/image_raw`
- **Format** : BGR8 converti via `cv_bridge`
- **LED** : verte si le flux est actif (données reçues dans les 3 dernières secondes)

---

## 5. Colonne droite

### 5.1 Carte ARUCO

Affiche la **position** et l'**orientation** mesurées par la vision ArUco :

```
Pos:  X=+4.80 cm   Y=-0.80 cm   Z=+0.90 cm
Ori:  R=+2.2°  P=+2.9°  Y=-4.0°
```

- **Position** : convertie de mètres en **centimètres** (×100), 2 décimales
- **Orientation** : en **degrés**, 1 décimale
- **Topics** : `aruco_position` (3 floats, en m) et `aruco_orientation` (3 floats, en °)

### 5.2 Carte IMU

Affiche les angles Roll, Pitch, Yaw mesurés par les centrales inertielles :

```
R: +2.50°   P: +1.80°   Y: +0.50°
```

- **Unité** : degrés (°), 2 décimales
- **Topic** : `imu_error`

### 5.3 Carte FUSION

Affiche les angles fusionnés (combinaison IMU + ArUco par filtre complémentaire) :

```
R: +2.30°   P: +1.50°   Y: -0.40°
```

- **Unité** : degrés (°), 2 décimales
- **Topic** : `F_orientation`

### 5.4 Carte MOTEURS

Affiche les **consignes** et le **feedback** des 6 vérins en temps réel :

```
C: V1:6.279 cm  V2:5.800 cm  V3:8.300 cm  V4:7.900 cm  V5:6.800 cm  V6:7.500 cm
F: V1:6.275 cm  V2:5.803 cm  V3:8.298 cm  V4:7.898 cm  V5:6.802 cm  V6:7.497 cm
```

- **C** (bleu) : consignes envoyées aux moteurs — topic `stewart/longueurs`
- **F** (orange) : positions réelles mesurées — topic `feedback_motors`
- **Unité** : centimètres (cm), 3 décimales

### 5.5 Graphes moteurs M1–M6

Six graphiques en temps réel (grille 3×2) affichent pour chaque vérin :

- **Courbe bleue** : consigne
- **Courbe rouge** : feedback (position réelle)
- **Axe Y** : longueur en cm (`L (cm)`)
- **Axe X** : échantillons (100 derniers points, glissant)

Les graphes permettent de visualiser instantanément :
- La précision du suivi de consigne
- Les oscillations ou dépassements
- Les retards de réponse entre consigne et feedback

---

## 6. Indicateurs LED

Chaque section capteur possède un **indicateur LED** qui reflète l'état de la connexion :

| Couleur | État | Signification |
|---|---|---|
| 🟢 Vert | `ok` | Données reçues dans les 3 dernières secondes |
| 🟡 Orange | `warn` | Avertissement (non utilisé actuellement) |
| 🔴 Rouge | `error` | Erreur critique (non utilisé actuellement) |
| ⚫ Gris | `off` | Aucune donnée depuis plus de 3 secondes |

Les LEDs sont mises à jour toutes les **2 secondes**.

**Capteurs surveillés :** ArUco, IMU, Fusion, Moteurs, Caméra.

---

## 7. Mode Démo (sans ROS2)

Le script de démonstration permet de tester l'interface **sans matériel ni ROS2** :

```bash
./run_demo.sh
```

**Caractéristiques :**
- Données sinusoïdales simulées à **20 Hz**
- Les 6 capteurs alimentés : IMU, ArUco (position + orientation), Fusion, Moteurs (consignes + feedback)
- Le feedback suit la consigne avec un **retard de 0.4 s** et un bruit gaussien
- L'indicateur **● DEMO MODE** (orange) apparaît dans la section Actions
- Les boutons d'action sont **désactivés** (grisés)

> 💡 Utile pour valider les modifications visuelles de l'interface, faire des captures d'écran ou des présentations.

---

## 8. Raccourcis et astuces

| Action | Méthode |
|---|---|
| Modifier une valeur rapidement | Cliquer sur le champ, taper la valeur, Entrée |
| Incrémenter finement | Utiliser les flèches ▲▼ du spinbox |
| Redimensionner | La fenêtre est redimensionnable (minimum 1100×680) |
| Plein écran | Double-cliquer sur la barre de titre (selon le gestionnaire de fenêtres) |

---

## 9. Dépannage

### `ModuleNotFoundError: No module named 'PySide6'`

PySide6 n'est pas installé pour le Python utilisé par ROS2 (Python système 3.12) :

```bash
/usr/bin/python3 -m pip install --user --break-system-packages PySide6
```

### Le logo ABMI ne s'affiche pas

Le fichier `abmi_logo.png` doit être présent dans l'un des emplacements suivants :
- `src/stewart_control/assets/abmi_logo.png` (source)
- `install/stewart_control/share/stewart_control/assets/abmi_logo.png` (après `colcon build`)

Si le logo est absent, le texte « ABMI » s'affiche à la place.

### Les LEDs restent grises

Aucune donnée n'est reçue depuis plus de 3 secondes. Vérifiez :
1. Que le mode **Acquisition** ou **Automatique** est lancé
2. Que les nœuds publient bien sur les topics attendus : `ros2 topic list`
3. Que le hardware (IMU, caméra) est connecté

### Les boutons n'ont aucun effet

- En **mode Démo**, les boutons sont désactivés (normal).
- En mode ROS2, vérifiez que `ros2` est accessible : `which ros2`
- Vérifiez les logs dans le terminal pour les erreurs de lancement.

### Les graphes n'affichent rien

Les graphes nécessitent la réception de données sur `stewart/longueurs` (consignes) ET `feedback_motors` (feedback). Lancez au minimum le mode **Acquisition**.

---

*Document généré pour le projet Stewart Platform — ABMI Groupe.*
