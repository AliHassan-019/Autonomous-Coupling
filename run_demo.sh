#!/usr/bin/env bash
# Lance la démo de l'interface Stewart Platform avec données simulées.
# Aucune dépendance ROS2 requise.
#
# Usage:  ./run_demo.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONPATH="${SCRIPT_DIR}/src/stewart_control:${PYTHONPATH}"

exec python3 "${SCRIPT_DIR}/src/stewart_control/test/demo_interface.py" "$@"
