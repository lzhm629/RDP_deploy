#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ -z "${CONDA_PREFIX:-}" ]]; then
    echo "No Conda environment is active."
    echo "Run: conda activate rdp_deploy"
    exit 1
fi

echo "Using Conda environment: ${CONDA_PREFIX}"
python -m pip install -r "${SCRIPT_DIR}/requirements.txt"
python "${SCRIPT_DIR}/scripts/check_imports.py" --scope core
python "${SCRIPT_DIR}/scripts/check_imports.py" --scope hardware

echo
echo "Core and hardware dependency checks completed."
