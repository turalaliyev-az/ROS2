#!/usr/bin/env bash
# The robot's own display: a full-screen page with the control panel's QR code
# and status. Started by the desktop session (autologin); Alt+F4 closes it.
URL="http://localhost:${ROVER_WEB_PORT:-8080}/qr"
PROFILE="$HOME/.rover/screen-browser"

until curl -fs -o /dev/null "$URL"; do sleep 2; done

# Robots lose power without a clean shutdown; stop Chrome from greeting the
# next boot with a "restore pages?" bubble on top of the QR code.
PREFS="$PROFILE/Default/Preferences"
if [ -f "$PREFS" ]; then
  sed -i 's/"exited_cleanly":false/"exited_cleanly":true/; s/"exit_type":"[^"]*"/"exit_type":"Normal"/' "$PREFS"
fi

exec google-chrome --kiosk --user-data-dir="$PROFILE" \
  --no-first-run --no-default-browser-check --noerrdialogs --disable-infobars \
  --disable-session-crashed-bubble --disable-translate --password-store=basic \
  "$URL"
