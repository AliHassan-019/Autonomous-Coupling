#!/usr/bin/env python3

"""Chargement centralisé de la configuration YAML."""

import os
import yaml

_DEFAULT_CONFIG_FILENAME = "stewart_params.yaml"

# Ordre de recherche :
# 1. Variable d'environnement STEWART_CONFIG
# 2. <package>/config/stewart_params.yaml  (développement)
# 3. Dossier share installé via colcon


def _find_config_file():
    """Recherche le fichier de configuration YAML."""

    # 1 — Variable d'environnement
    env = os.environ.get("STEWART_CONFIG")
    if env and os.path.isfile(env):
        return env

    # 2 — À côté du paquet source (développement)
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    dev_path = os.path.join(pkg_dir, "config", _DEFAULT_CONFIG_FILENAME)
    if os.path.isfile(dev_path):
        return dev_path

    # 3 — Installé via colcon (share)
    try:
        from ament_index_python.packages import get_package_share_directory

        share = get_package_share_directory("stewart_control")
        share_path = os.path.join(share, "config", _DEFAULT_CONFIG_FILENAME)
        if os.path.isfile(share_path):
            return share_path
    except Exception:
        pass

    raise FileNotFoundError(
        f"Configuration YAML introuvable. "
        f"Définissez STEWART_CONFIG ou placez {_DEFAULT_CONFIG_FILENAME} "
        f"dans le dossier config/ du paquet."
    )


def load_config(config_path=None):
    """
    Charge et retourne le dictionnaire de configuration.

    Args:
        config_path: Chemin explicite vers le fichier YAML.
                     Si None, recherche automatique.

    Returns:
        dict: Configuration complète.
    """
    if config_path is None:
        config_path = _find_config_file()

    with open(config_path, "r") as f:
        cfg = yaml.safe_load(f)

    return cfg


def get_config_path():
    """Return the resolved YAML configuration path."""

    return _find_config_file()


def save_config(cfg, config_path=None):
    """Persist the configuration dictionary back to YAML."""

    if config_path is None:
        config_path = _find_config_file()

    with open(config_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(cfg, f, sort_keys=False)


# Cache global (chargé une seule fois par processus)
_config_cache = None


def get_config(config_path=None):
    """
    Retourne la configuration, avec mise en cache.

    Args:
        config_path: Chemin explicite (force le rechargement).

    Returns:
        dict: Configuration complète.
    """
    global _config_cache
    if _config_cache is None or config_path is not None:
        _config_cache = load_config(config_path)
    return _config_cache
