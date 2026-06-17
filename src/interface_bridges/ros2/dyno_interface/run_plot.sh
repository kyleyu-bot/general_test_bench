#!/bin/bash
# Launch the Dyno Live Plot window.
# Run from the repo root: bash src/interface_bridges/ros2/run_plot.sh
#
# Mirrors run_gui.sh: forces UDP DDS transport so it can communicate with
# bridge_ros2 running as root without shared-memory permission issues.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROFILES="$SCRIPT_DIR/fastdds_no_shm.xml"

source /opt/ros/humble/setup.bash

export FASTRTPS_DEFAULT_PROFILES_FILE="$PROFILES"

# Include user's local pip packages so pyqtgraph etc. are visible under sudo.
USER_SITE="$(python3 -m site --user-site 2>/dev/null || true)"
if [ -n "$USER_SITE" ]; then
    export PYTHONPATH="$USER_SITE${PYTHONPATH:+:$PYTHONPATH}"
fi

exec sudo \
    PYTHONPATH="$PYTHONPATH" \
    FASTRTPS_DEFAULT_PROFILES_FILE="$PROFILES" \
    ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    python3 "$SCRIPT_DIR/dyno_plot.py" "$@"
