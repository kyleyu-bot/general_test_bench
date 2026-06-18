#!/bin/bash
# Run once to install the Joint Impact Prevention Testbench desktop launcher.
# Usage: bash desktop_executables/install_joint_impact_prevention_testbench.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LAUNCHER="$SCRIPT_DIR/launch_joint_impact_prevention_testbench.sh"
DESKTOP_FILE="$HOME/Desktop/joint_impact_prevention_testbench.desktop"

chmod +x "$LAUNCHER"

cat > "$DESKTOP_FILE" <<EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=Joint Impact Prevention Testbench
Exec=bash $LAUNCHER
Icon=applications-science
Terminal=false
Categories=Science;Engineering;
EOF

chmod +x "$DESKTOP_FILE"

# Suppress the GNOME "untrusted application" dialog.
if command -v gio &>/dev/null; then
    gio set "$DESKTOP_FILE" metadata::trusted true 2>/dev/null || true
fi

echo "Installed: $DESKTOP_FILE"
