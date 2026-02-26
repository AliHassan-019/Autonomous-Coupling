# stewart_control ROS2 Project

[![CI - Stewart Control](https://github.com/jbernardo6u/stewart-control_ws/actions/workflows/ci.yml/badge.svg)](https://github.com/jbernardo6u/stewart-control_ws/actions/workflows/ci.yml)

Ce dépôt contient un workspace ROS2 (`colcon`) dédié au contrôle d'une plateforme Stewart.

## Structure

- `src/stewart_control` : paquet principal contenant les noeuds ROS et le code de cinématique.
- `src/stewart_control/Arduino` : code embarqué pour la carte contrôlant les vérins.

## Compilation

```bash
# depuis la racine du workspace
eval "$(ros2 env)"          # source l'environnement ROS2
colcon build --symlink-install
```

## Exécution

```bash
# lancer les noeuds de capteurs et fusion
ros2 launch stewart_control stewart_ordered_launch.py

# ensuite démarrer stewart_node ou manual_stewart_node
ros2 run stewart_control stewart_node
```

## Tests

Le projet inclut des tests unitaires pour les modules critiques :
- Cinématique inverse (`test_inv_kinematics.py`)
- Filtre de Kalman et fusion (`test_fusion.py`)

### Lancer les tests

```bash
# Installer pytest si nécessaire
pip install --user pytest

# Lancer tous les tests fonctionnels
python -m pytest src/stewart_control/test/test_inv_kinematics.py \
                 src/stewart_control/test/test_fusion.py -v

# Lancer un fichier de test spécifique
python -m pytest src/stewart_control/test/test_inv_kinematics.py -v

# Avec rapport de couverture (optionnel)
pip install --user pytest-cov
python -m pytest src/stewart_control/test/ --cov=stewart_control
```

**Note** : Les tests ament (copyright, flake8, pep257) nécessitent un environnement ROS2 complet.

## Branches Git

- `main` : version stable
- `develop` : intégration des fonctionnalités en cours
- branches de fonctionnalités pour chaque ticket

## Qualité du code

Ce projet utilise les outils suivants :

* **Black** pour le formatage Python (ligne max 88 caractères)
* **flake8** pour le linting Python
* **pre-commit** pour lancer automatiquement ces vérifications avant chaque commit

### Installation

Après avoir cloné le dépôt :

```bash
# Installer les dépendances
pip install --user pre-commit black flake8

# Installer les hooks git
pre-commit install

# Lancer les vérifications sur tous les fichiers (première fois)
pre-commit run --all-files
```

### Utilisation quotidienne

Les hooks s'exécutent automatiquement avant chaque commit. Si des erreurs sont détectées :

1. **Black/flake8** : corrections automatiques appliquées, vérifiez et re-commitez
2. Le commit est bloqué si des corrections doivent être faites manuellement

Pour lancer manuellement les vérifications :

```bash
# Sur tous les fichiers
pre-commit run --all-files

# Sur des fichiers spécifiques
pre-commit run --files src/stewart_control/stewart_control/mon_fichier.py

# Bypasser les hooks (déconseillé)
git commit --no-verify -m "message"
```

## Contributions

1. Créez une nouvelle branche `feature/...` ou `bugfix/...`.
2. Formatez le code (`black`, `clang-format` pour Arduino).
3. Ajoutez/modifiez des tests si nécessaire.
4. Faites une pull request.

---

*Documentation basique créée le 26 février 2026.*
