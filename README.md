# Projet d'attelage automatique

Ce projet a pour objectif de développer un système d’attelage automatique entre  un tracteur et une remorque grâce au pilotage d’une plateforme de mouvement à six degrés de liberté (6 DOF).

Le système combine plusieurs capteurs  pour estimer précisément la position relative des deux éléments et ajuste en temps réel les mouvements de la plateforme afin d’aligner et d’atteler les véhicules de façon autonome et précise.

Le traitement des données et la logique de contrôle sont répartis entre une Raspberry Pi (calcul et traitement des capteurs) et une carte Arduino (commande des moteurs).

## Matériel utilisé

- Raspberry Pi 5  
- Arduino Mega
- 6 moteurs JGA25370
- 6 Driver DRV8871
- Caméra avec marqueurs ArUco   
- Capteurs IMU
- Ecran tactile
- plateforme de mouvement(coté mécanique)

## Rôle de chaque carte
**Raspberry Pi 5** :

- Gère les capteurs haut niveau (caméra, IMU, GPS..)

- Calcule les consignes de mouvement 

- Envoie les consignes vers l’Arduino via UART

**Arduino Mega** :

- Reçoit les consignes via UART3

- Pilote les 6 moteurs via les drivers DRV8871

- Gère le contrôle bas niveau (PWM, direction, vitesse)



  ##  État actuel du projet

-  Communication UART entre la Raspberry Pi et l’Arduino Mega 
-  Envoi de consignes depuis la Raspberry Pi vers l’Arduino
-  Code fonctionnel pour la lecture de deux capteurs IMU et le calcul de l’orientation relative

  

