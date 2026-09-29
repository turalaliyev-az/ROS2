#!/usr/bin/env bash
# Makes the rover come up by itself on power-on:
#   - systemd user service "rover": motors, sensors, Nav2 and the web panel
#     (starts at boot, before anyone logs in)
#   - desktop autostart: the full-screen QR code on the robot's screen
#
# Usage: install_autostart.sh            install and enable
#        install_autostart.sh --remove   undo
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$HERE/../../.." && pwd)"
UNIT_DIR="$HOME/.config/systemd/user"
AUTOSTART="$HOME/.config/autostart/rover-screen.desktop"

if [ "${1:-}" = "--remove" ]; then
  systemctl --user disable --now rover.service 2>/dev/null || true
  rm -f "$UNIT_DIR/rover.service" "$AUTOSTART"
  systemctl --user daemon-reload
  echo "Autostart removed."
  exit 0
fi

mkdir -p "$UNIT_DIR" "$(dirname "$AUTOSTART")"
sed "s|@WS@|$WS|g" "$HERE/rover.service" > "$UNIT_DIR/rover.service"
sed "s|@WS@|$WS|g" "$HERE/rover-screen.desktop" > "$AUTOSTART"
chmod +x "$HERE/rover-screen.sh"

# User services normally start at login; linger starts them at boot.
if [ "$(loginctl show-user "$USER" -p Linger --value)" != "yes" ]; then
  loginctl enable-linger "$USER"
fi

systemctl --user daemon-reload
systemctl --user enable rover.service
echo "Installed: the rover starts on every boot."
echo "  start now : systemctl --user start rover"
echo "  stop      : systemctl --user stop rover"
echo "  logs      : journalctl --user -u rover -f"
