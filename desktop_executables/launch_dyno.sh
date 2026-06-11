#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

mkdir -p "$HOME/.local/bin"

# Askpass for realtime kernel setup (runs only when tuning needs (re)applying).
# Always rewritten so text changes propagate to existing installs.
RT_ASKPASS="$HOME/.local/bin/dyno-askpass-rt"
cat > "$RT_ASKPASS" <<'EOF'
#!/bin/bash
zenity --password \
    --title="Dyno — Realtime Setup" \
    --text="Enter password to configure realtime kernel settings.\nThis prompt appears when realtime tuning needs to be (re)applied." \
    2>/dev/null
EOF
chmod +x "$RT_ASKPASS"

# Askpass for EtherCAT bridge launch.
GUI_ASKPASS="$HOME/.local/bin/dyno-askpass-gui"
if [ ! -f "$GUI_ASKPASS" ]; then
    cat > "$GUI_ASKPASS" <<'EOF'
#!/bin/bash
zenity --password \
    --title="Dyno — EtherCAT Bridge" \
    --text="Enter password to start the EtherCAT bridge:" \
    2>/dev/null
EOF
    chmod +x "$GUI_ASKPASS"
fi

cd "$REPO_ROOT"

# Verify realtime tuning at every launch; heal it only when something is off
# (setup skipped because the NIC wasn't up yet at boot, governor reverted by
# power-profiles-daemon, irqbalance re-enabled, ...).  No prompt when tuned.
# env_setup.sh runs as a single sudo call so its internal sudo calls run as
# root and never prompt again.
CHECK_RT="$REPO_ROOT/env_setup_scripts/check_rt_env.sh"
if ! bash "$CHECK_RT"; then
    export SUDO_ASKPASS="$RT_ASKPASS"
    sudo -A bash "$REPO_ROOT/env_setup_scripts/env_setup.sh" || true
    sleep 2  # let NIC/IRQ changes stabilize before starting EtherCAT
    if ! bash "$CHECK_RT"; then
        zenity --warning --title="Dyno — Realtime Setup" \
            --text="Realtime tuning could not be fully applied.\nEtherCAT cycle timing may be degraded — see terminal output." \
            2>/dev/null || true
    fi
fi

# Kill any leftover root processes from a previous run.
# This sudo call also warms the credential cache so run_gui.sh's sudo is silent.
export SUDO_ASKPASS="$GUI_ASKPASS"
sudo -A pkill -f bridge_ros2 2>/dev/null || true
sudo -A pkill -f dyno_gui.py  2>/dev/null || true
sudo -A chown -R "$USER" "$REPO_ROOT/test_data_log" 2>/dev/null || true
sleep 0.5

exec bash "$REPO_ROOT/src/interface_bridges/ros2/run_gui.sh"
