#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
RT_STATUS_FILE="/tmp/.dyno_rt_setup_${USER}"

mkdir -p "$HOME/.local/bin"

# Askpass for realtime kernel setup (runs once per boot).
RT_ASKPASS="$HOME/.local/bin/dyno-askpass-rt"
if [ ! -f "$RT_ASKPASS" ]; then
    cat > "$RT_ASKPASS" <<'EOF'
#!/bin/bash
zenity --password \
    --title="Dyno — Realtime Setup" \
    --text="Enter password to configure realtime kernel settings.\nThis prompt appears once per boot." \
    2>/dev/null
EOF
    chmod +x "$RT_ASKPASS"
fi

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

# Run realtime environment setup once per boot.
# /tmp is cleared on reboot so this naturally re-runs after each restart.
# Run as a single sudo call so internal sudo calls inside env_setup.sh run
# as root and never prompt again.
if [ ! -f "$RT_STATUS_FILE" ]; then
    export SUDO_ASKPASS="$RT_ASKPASS"
    sudo -A bash "$REPO_ROOT/env_setup_scripts/env_setup.sh"
    touch "$RT_STATUS_FILE"
    sleep 2  # let NIC/IRQ changes stabilize before starting EtherCAT
fi

# Kill any leftover root processes from a previous run.
# This sudo call also warms the credential cache so run_gui.sh's sudo is silent.
export SUDO_ASKPASS="$GUI_ASKPASS"
sudo -A pkill -f bridge_ros2 2>/dev/null || true
sudo -A pkill -f dyno_gui.py  2>/dev/null || true
sudo -A chown -R "$USER" "$REPO_ROOT/test_data_log" 2>/dev/null || true
sleep 0.5

exec bash "$REPO_ROOT/src/interface_bridges/ros2/run_gui.sh"
