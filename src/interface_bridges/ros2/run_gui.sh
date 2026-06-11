#!/bin/bash
# Launch the Dyno Qt GUI (and bridge_ros2 subprocess).
# Self-contained: can be invoked directly as a normal user or via launch_dyno.sh.
#
# Why this wrapper exists:
#   bridge_ros2 must run as root (raw EtherCAT socket).  Fast-DDS shared memory
#   segments are not accessible across root/user boundaries, causing silent
#   message drops.  Forcing UDP transport on both sides fixes this.
#   PYTHONPATH must also be forwarded so rclpy is visible under sudo.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
PROFILES="$SCRIPT_DIR/fastdds_no_shm.xml"

source /opt/ros/humble/setup.bash

export FASTRTPS_DEFAULT_PROFILES_FILE="$PROFILES"

# Include user's local pip packages so PyQt5 etc. are visible under sudo.
USER_SITE="$(python3 -m site --user-site 2>/dev/null || true)"
if [ -n "$USER_SITE" ]; then
    export PYTHONPATH="$USER_SITE${PYTHONPATH:+:$PYTHONPATH}"
fi

# All relative paths in dyno_gui.py (topology, bridge binary, error/register maps)
# are resolved from cwd — always run from the repo root.
cd "$REPO_ROOT"

# Create a zenity-based askpass helper for the EtherCAT bridge sudo prompt.
GUI_ASKPASS="$HOME/.local/bin/dyno-askpass-gui"
if [ ! -f "$GUI_ASKPASS" ]; then
    mkdir -p "$HOME/.local/bin"
    cat > "$GUI_ASKPASS" <<'EOF'
#!/bin/bash
zenity --password \
    --title="Dyno — EtherCAT Bridge" \
    --text="Enter password to start the EtherCAT bridge:" \
    2>/dev/null
EOF
    chmod +x "$GUI_ASKPASS"
fi
export SUDO_ASKPASS="$GUI_ASKPASS"

# Kill any leftover root processes that would hold the EtherCAT socket.
sudo -A pkill -f bridge_ros2 2>/dev/null || true
sudo -A pkill -f dyno_gui.py  2>/dev/null || true
sleep 0.3

# Verify realtime tuning at every launch (NIC/IRQ pinning, CPU governor,
# irqbalance); heal it only when something is off.  No prompt when tuned.
# When invoked via launch_dyno.sh the environment is already healed, so this
# check passes instantly; standalone invocations stay self-contained.
CHECK_RT="$REPO_ROOT/env_setup_scripts/check_rt_env.sh"
if ! bash "$CHECK_RT"; then
    RT_ASKPASS="$HOME/.local/bin/dyno-askpass-rt"
    mkdir -p "$HOME/.local/bin"
    cat > "$RT_ASKPASS" <<'EOF'
#!/bin/bash
zenity --password \
    --title="Dyno — Realtime Setup" \
    --text="Enter password to configure realtime kernel settings.\nThis prompt appears when realtime tuning needs to be (re)applied." \
    2>/dev/null
EOF
    chmod +x "$RT_ASKPASS"
    SUDO_ASKPASS="$RT_ASKPASS" sudo -A bash "$REPO_ROOT/env_setup_scripts/env_setup.sh" || true
    sleep 2
    if ! bash "$CHECK_RT"; then
        zenity --warning --title="Dyno — Realtime Setup" \
            --text="Realtime tuning could not be fully applied.\nEtherCAT cycle timing may be degraded — see terminal output." \
            2>/dev/null || true
    fi
fi

sudo -A \
    PYTHONPATH="$PYTHONPATH" \
    FASTRTPS_DEFAULT_PROFILES_FILE="$PROFILES" \
    ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    python3 "$SCRIPT_DIR/dyno_gui.py" "$@"
