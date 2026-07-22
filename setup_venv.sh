#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
VENV_DIR="${REPO_DIR}/rdp_deploy_venv"
PYTHON_BIN="${PYTHON_BIN:-python3.12}"

"${PYTHON_BIN}" -m venv "${VENV_DIR}"
source "${VENV_DIR}/bin/activate"
python -m pip install --upgrade pip
python -m pip install -r "${SCRIPT_DIR}/requirements.txt"

echo "Created venv at ${VENV_DIR}"
echo "Before running deployment scripts:"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  source ${VENV_DIR}/bin/activate"
echo "  export PYTHONPATH=${REPO_DIR}/reactive_diffusion_policy:${SCRIPT_DIR}:\$PYTHONPATH"
