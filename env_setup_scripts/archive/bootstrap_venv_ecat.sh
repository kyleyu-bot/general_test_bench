#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="${REPO_ROOT}/.venv-ecat"
PYTHON_BIN="${PYTHON_BIN:-python3}"
CAPS="${CAPS:-cap_net_raw,cap_net_admin,cap_sys_nice}"
VENV_PYTHON="${VENV_DIR}/bin/python"

echo "Repo root: ${REPO_ROOT}"
echo "Venv dir:  ${VENV_DIR}"
echo "Python:    ${PYTHON_BIN}"

if ! command -v "${PYTHON_BIN}" >/dev/null 2>&1; then
    echo "error: Python interpreter not found: ${PYTHON_BIN}" >&2
    exit 1
fi

if [[ ! -d "${VENV_DIR}" ]]; then
    echo "Creating virtual environment"
    "${PYTHON_BIN}" -m venv --copies "${VENV_DIR}"
else
    echo "Using existing virtual environment"
fi

if [[ ! -x "${VENV_PYTHON}" ]]; then
    echo "Virtual environment is incomplete: missing ${VENV_PYTHON}" >&2
    echo "Remove ${VENV_DIR} and rerun this script after installing python3-venv." >&2
    exit 1
fi

if ! "${VENV_PYTHON}" -m pip --version >/dev/null 2>&1; then
    echo "Virtual environment is incomplete: pip is unavailable in ${VENV_DIR}" >&2
    echo "Remove ${VENV_DIR} and rerun this script after installing python3-venv." >&2
    exit 1
fi

echo "Upgrading pip"
"${VENV_PYTHON}" -m pip install --upgrade pip

echo "Installing pysoem"
"${VENV_PYTHON}" -m pip install pysoem

cat <<EOF

Bootstrap complete.

This script uses \`python -m venv --copies\` so ${VENV_DIR}/bin/python3 is a
regular file. That is required for \`setcap\`; it will fail on symlink-based
venvs.

Activate it with:
  source "${VENV_DIR}/bin/activate"

Grant default EtherCAT and RT scheduling capabilities to this venv interpreter with:
  sudo "${REPO_ROOT}/env_setup_scripts/enable_ethercat_caps.sh"

Verify capabilities with:
  getcap "${VENV_DIR}/bin/python3"

Then run tests without sudo, for example:
  "${REPO_ROOT}/env_setup_scripts/run_ethercat_python.sh" src/tools/scan_pysoem.py --iface ecat0

If you need a narrower capability set than the default, rerun the capability helper with:
  sudo "${REPO_ROOT}/env_setup_scripts/enable_ethercat_caps.sh" --caps cap_net_raw,cap_sys_nice

If a specific host still requires full elevation, use:
  "${REPO_ROOT}/env_setup_scripts/run_ethercat_python.sh" --sudo <script> [args]
EOF
