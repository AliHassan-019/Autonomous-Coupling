# Projet d'attelage autonome

Ce projet a pour objectif de développer un système d’attelage automatique entre  un tracteur et une remorque grâce au pilotage d’une plateforme de mouvement à six degrés de liberté (6 DOF).

Le système combine plusieurs capteurs  pour estimer précisément la position relative des deux éléments et ajuste en temps réel les mouvements de la plateforme afin d’aligner et d’atteler les véhicules de façon autonome et précise.

Le traitement des données et la logique de contrôle sont répartis entre une Raspberry Pi (calcul et traitement des capteurs) et une carte Arduino (commande des moteurs).

## 🛠️ Matériel utilisé
| Matériel                | Rôle                                                      |
|------------------------|-----------------------------------------------------------|
| **Raspberry Pi 5**     | Exécute ROS 2 Jazzy, traite vision et IMU, calcule consignes. |
| **Arduino Mega**       | Pilote les 6 moteurs (PWM, direction, vitesse).           |
| **6 moteurs JGA25370** | Actionnement de la plateforme Stewart.                    |
| **6 drivers DRV8871**  | Contrôle des moteurs DC.                                  |
| **Caméra + ArUco**     | Détection de position et orientation.                     |
| **Capteurs IMU**       | Mesure orientation relative.                              |
| **Écran tactile**      | Interface utilisateur pour contrôle et monitoring.        |
| **Plateforme mécanique**| Plateforme Stewart (6 vérins). ( pas encore prete )  


## Rôle de chaque carte
**Raspberry Pi 5** :

- Gère les capteurs haut niveau (caméra, IMU, GPS..)

- Calcule les consignes de mouvement 

- Envoie les consignes vers l’Arduino via UART

**Arduino Mega** :

- Reçoit les consignes via UART3

- Pilote les 6 moteurs via les drivers DRV8871

- Gère le contrôle bas niveau (PWM, direction, vitesse)




## 🧩 Architecture générale

![Architecture](docs/architecture.png)

Le projet s’articule autour de **ROS 2 Jazzy** (sur Raspberry Pi) et d’un **Arduino** pour le contrôle bas niveau.

### 🔹 Flux de données :
1. `Interface_node` est le **point d’entrée** :
   - Un clic sur **Commencer** démarre automatiquement :
     - `aruco_node` (vision)
     - `IMU_node` (orientation)
     - `cinematique_inverse` (calcul longueurs vérins)
2. Les données de **position** (`/aruco_positions`) et d’**orientation** (`/IMU_error`) sont reçues en temps réel.
3. `cinematique_inverse` calcule les longueurs et publie `/longueurs_verins`.
4. L’interface affiche **toutes les données en live**.
5. Le bouton **Arrêter** stoppe tout sauf l’interface.


### 🔹 Nœuds ROS 2
| Nœud                 | Rôle                                                                    |
|----------------------|----------------------------------------------------------------------   |
| `Interface_node`     | Interface graphique, gestion du système et visualisation en temps réel. |
| `aruco_node`         | Détection des marqueurs ArUco, publication `/aruco_positions`.          |
| `IMU_node`           | Lecture et publication des données IMU `/IMU_error`.                    |
| `stewart_node`       | Calcul des longueurs vérins à partir des positions/orientations.        |
| `Programme_moteurs`  | Code Arduino exécutant les consignes moteurs envoyées par la pi via
                         UART3                                                                   |


### 🔹 Topics ROS 2
| Topic                | Publié par              | Contenu                                    |
|----------------------|------------------------|-------------------------------------------|
| `/aruco_positions`   | `aruco_node`           | Position (x, y, z).                       |
| `/IMU_error`         | `IMU_node`             |  orientation IMU (roll, yaw , pitch).     |
| `/longueurs/verins`  | `stewart_node`         | Longueurs calculées pour chaque vérin.    |







<img width="1619" height="964" alt="image" src="https://github.com/user-attachments/assets/4354006b-75d7-4ca8-85bc-1d1d9ccaf691" />










##🚀 Utilisation

1-Lancer uniquement l’interface :
ros2 run stewart_control interface_node

2-Cliquer sur Commencer dans l’interface :
→ Les nœuds aruco_node, IMU_node, cinematique_inverse se lancent.
→ Les données sont affichées en temps réel.

3-Cliquer sur Arrêter :
→ Equivalent à Ctrl+C, stoppe les nœuds sauf l’interface.



  ##  État actuel du projet

-  Communication UART entre la Raspberry Pi et l’Arduino Mega 
-  Envoi de consignes depuis la Raspberry Pi vers l’Arduino
-  Code fonctionnel pour la lecture de deux capteurs IMU et le calcul de l’orientation relative

  

