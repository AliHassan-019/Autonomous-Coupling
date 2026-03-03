# EPM

Ce document synthétise la démarche expérimentale EPM et intègre les derniers résultats de force de maintien.

## Contexte
- Objectif : piloter l'EPM en ON/OFF avec une commutation reproductible.
- Besoin : impulsion de courant intense et brève, en contrôlant tension, durée, énergie et impédance série.

## Hypothèses initiales
- Inductance supposée : `L ≈ 35 µH`.
- Cible courant : `35 A` sous `15 V`.
- Estimation initiale : `t = L*I/V ≈ 82 µs`.
- Estimation énergie condensateur (470 µF, 15 V) : `≈ 53 mJ`.

## Limite de la méthode RLC
- Méthode initiale : extraction de `L` via oscillations RLC.
- Constat : pas d'oscillation exploitable.
- Cause : résistance série trop élevée, régime sur-amorti.
- Décision : abandon RLC et bascule sur approche RL.

## Approche RL retenue
- Formule : `L = V * (Δt / ΔI)`.
- Mesure de la pente de courant avec shunt + oscilloscope.
- Shunt improvisé par fil calibré : 50 cm à 0,23 Ω puis coupe à ~2,2 cm pour viser ~10 mΩ.

## Montage expérimental
- Chaîne : alimentation → condensateur → BTS7960 → shunt → bobine EPM.
- Instrumentation :
  - CH1 oscilloscope sur bobine,
  - CH2 sur shunt.
- Commande : Arduino via RPWM/LPWM.

## Résultats électriques
- Inductance mesurée : `L ≈ 10 µH`.
- Temps théorique pour 35 A à 29 V : `t ≈ 12 µs`.
- Résistance de charge : `R_charge ≈ 7,2 Ω`.
- Temps de recharge : `T_charge ≈ 36 ms`.
- Énergie magnétique : `≈ 6,1 mJ`.
- Énergie condensateur :
  - 1000 µF à 29 V : `≈ 420 mJ`,
  - avec ajout de 3x470 µF : `≈ 1,19 J`.

## Problèmes rencontrés
- À 15 V : maintien insuffisant, relâchement rapide.
- Échauffement avec fils fins / montage non robuste.
- Chauffe BTS7960 si impulsion trop longue.
- Faux contacts lors des essais.

## Solutions appliquées
- Passage à `29 V` pour fiabiliser le maintien.
- Augmentation capacité (470 µF → 1000 µF + ajout de capacités).
- Pilotage par impulsion unique avec logique de sécurité.

## Force de maintien
### Campagne 1 (objets)
- Zone de maintien observée autour de `25–28 N`.
- Décrochage observé vers `31 N`.

### Campagne 2 (banc dynamomètre, 10 mesures)
- Masse maximale moyenne : `2,07 kg`.
- Force moyenne : `20,4 N`.

> Valeur de référence retenue actuellement : **20,4 N** (mesure banc instrumenté), plus fiable que les tests objets.

## Conclusion
- `L ≈ 10 µH` confirmée expérimentalement.
- Commutation ON/OFF validée avec architecture BTS7960 + impulsion contrôlée.
- Le dimensionnement énergétique est suffisant ; la limitation pratique vient surtout de l'impédance série, du driver et des conditions mécaniques de contact.
- La force de maintien nominale à retenir pour l'état actuel du prototype est `≈ 20,4 N`.

## Pistes de suite
- Étudier les paramètres augmentant la force de maintien (matériaux, géométrie de contact, surface d'appui).
- Industrialiser le banc de mesure (poulie guidée, montage mécanique rigide, répétabilité accrue).

---

## Explication technique du code EPM final

Le sketch Arduino de référence est :
- `epm_bts7960_test.ino`

Points essentiels :
1. `REN` et `LEN` restent à `HIGH` pour garder le pont en H actif.
2. Une seule direction est commandée à la fois (`RPWM` ou `LPWM`).
3. `etatRepos()` coupe systématiquement les PWM avant inversion.
4. La commande série permet `droite`, `gauche`, `test`, `pulse=NNN`.

Cette logique permet un basculement ON/OFF stable de l'EPM et limite les risques de commutation dangereuse.
