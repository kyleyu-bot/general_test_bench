#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_PYTHON="${REPO_ROOT}/.venv-ecat/bin/python"
USE_SUDO=0

usage() {
    cat <<EOF
Usage: $0 [--sudo] <script> [args...]

Run a Python script with the repo-local EtherCAT virtual-environment interpreter.

Options:
  --sudo       Run the venv interpreter with sudo.
  -h, --help   Show this help text.

Examples:
  $0 src/tools/scan_pysoem.py --iface ecat0
  $0 --sudo src/ecat_functional_tests/elm3002_pdo_check.py 2 --rt-priority 90 --cpu-affinity 0 --duration 600
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --sudo)
            USE_SUDO=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        --)
            shift
            break
            ;;
        -*)
            echo "error: unknown argument: $1" >&2
            usage >&2
            exit 1
            ;;
        *)
            break
            ;;
    esac
done

if [[ $# -lt 1 ]]; then
    usage >&2
    exit 1
fi

if [[ ! -x "${VENV_PYTHON}" ]]; then
    echo "error: venv interpreter not found: ${VENV_PYTHON}" >&2
    echo "Run ${REPO_ROOT}/env_setup_scripts/bootstrap_venv_ecat.sh first." >&2
    exit 1
fi

SCRIPT="$1"
shift

if [[ ${USE_SUDO} -eq 1 ]]; then
    exec sudo "${VENV_PYTHON}" "${SCRIPT}" "$@"
fi

exec "${VENV_PYTHON}" "${SCRIPT}" "$@"
