#!/bin/bash
# Run once to install the Dyno Testbench desktop launcher.
# Usage: bash desktop_executables/install_desktop.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LAUNCHER="$SCRIPT_DIR/launch_dyno.sh"
DESKTOP_FILE="$HOME/Desktop/dyno_testbench.desktop"

chmod +x "$LAUNCHER"

cat > "$DESKTOP_FILE" <<EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=Dyno Testbench
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
