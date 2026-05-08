#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"

echo "Batman-adv startup sequence initiated..."
echo "Waiting 5 seconds..."
sleep 5

echo "Running disable script..."
"$SCRIPT_DIR/disable_BATMAN_WiFi.sh"

echo "Waiting 5 seconds..."
sleep 5

echo "Running enable script..."
"$SCRIPT_DIR/enable_BATMAN_WiFi.sh"

sleep 5
echo "Restarting chrony for time synchonization..."
systemctl restart chrony

echo "Startup sequence complete."