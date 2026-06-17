#!/usr/bin/env bash
# Note: intentionally no set -e so that sourcing this script does not kill
# the calling shell if a step fails.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
    cat <<EOF
Usage: $0

Run the repo's environment setup scripts in sequence.

This wrapper runs:
  1. tune_realtime.sh
  2. rt_setup_part2.sh

Note: bootstrap_venv_ecat.sh and enable_ethercat_caps.sh are skipped —
Python/pysoem is no longer used; C++ binaries run with sudo directly.

It intentionally does not run run_ethercat_python.sh because that script is a
Python launcher for the repo-local venv, not an environment configuration step.
EOF
}

if [[ $# -gt 0 ]]; then
    case "$1" in
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "error: unknown argument: $1" >&2
            usage >&2
            exit 1
            ;;
    esac
fi

run_step() {
    local label="$1"
    shift

    echo
    echo "==> ${label}"
    "$@"
}

# Python/pysoem venv no longer needed — C++ stack uses SOEM directly.
# run_step "Bootstrapping EtherCAT virtual environment" \
#     "${SCRIPT_DIR}/bootstrap_venv_ecat.sh"

# C++ binaries are run with sudo directly — setcap on a Python interpreter
# is no longer needed.
# run_step "Applying interpreter capabilities" \
#     sudo "${SCRIPT_DIR}/enable_ethercat_caps.sh"

run_step "Applying host-level realtime tuning" \
    sudo "${SCRIPT_DIR}/tune_realtime.sh"

run_step "Applying NIC/IRQ realtime setup" \
    "${SCRIPT_DIR}/rt_setup_part2.sh"

echo
echo "Environment setup complete."

# ── ROS2 environment ──────────────────────────────────────────────────────────
# These only take effect in the calling shell when this script is sourced:
#   source env_setup_scripts/env_setup.sh
# They have no effect when the script is executed as a subprocess (./env_setup.sh).

REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

if [[ -f /opt/ros/humble/setup.bash ]]; then
    # shellcheck source=/dev/null
    source /opt/ros/humble/setup.bash
    export FASTRTPS_DEFAULT_PROFILES_FILE="${REPO_ROOT}/src/interface_bridges/ros2/dyno_interface/fastdds_no_shm.xml"
    echo "ROS2 Humble sourced. FASTRTPS_DEFAULT_PROFILES_FILE set."
    echo "Run the ROS2 bridge with: sudo -E ./build/dyno_ros2_bridge/bridge_ros2"
else
    echo "ROS2 Humble not found at /opt/ros/humble — skipping ROS2 setup."
fi
