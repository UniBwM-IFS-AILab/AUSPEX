#!/usr/bin/env bash
set -euo pipefail

# ============================================================================
# Batman-adv mesh network teardown script
# Unified script for Desktop Linux and Raspberry Pi
# Reverts changes made by enable_BATMAN_WiFi.sh
# ============================================================================

# Absolute path to the directory containing this script
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"

# Config file relative to the script
CONFIG_FILE="$SCRIPT_DIR/../../../auspex_params/platform_properties/platform_properties.json"

# Check if jq is installed
if ! command -v jq >/dev/null 2>&1; then
  echo "ERROR: jq is not installed. Install it (e.g. apt install jq)" >&2
  exit 1
fi

# Check if config file exists
if [ ! -f "$CONFIG_FILE" ]; then
  echo "ERROR: Configuration file not found: $CONFIG_FILE" >&2
  exit 1
fi

echo "Loading configuration from $CONFIG_FILE"

# Read configuration from JSON file
PLATFORM_ID=$(jq -r '.platform_id' "$CONFIG_FILE")
MESH_IFACE=$(jq -r '.network.mesh_interface' "$CONFIG_FILE")
BAT_IFACE=$(jq -r '.network.batman_interface' "$CONFIG_FILE")
OFFBOARD_CONTROLLER=$(jq -r '.platform_details.offboard_controller' "$CONFIG_FILE")

echo "Platform ID:        $PLATFORM_ID"
echo "Mesh Interface:     $MESH_IFACE"
echo "Batman Interface:   $BAT_IFACE"
echo "Offboard Controller: $OFFBOARD_CONTROLLER"
echo

echo "Running batman-adv mesh teardown..."

# Require root
if [ "$(id -u)" -ne 0 ]; then
  echo "ERROR: must run as root (use sudo)" >&2
  exit 1
fi

# Validate offboard controller
if [[ "$OFFBOARD_CONTROLLER" != "DESKTOP" && "$OFFBOARD_CONTROLLER" != "PI" ]]; then
  echo "ERROR: Invalid OFFBOARD_CONTROLLER value: $OFFBOARD_CONTROLLER" >&2
  echo "Supported values: DESKTOP, PI" >&2
  exit 1
fi

# Common tool checks
for cmd in iw ip batctl; do
  if ! command -v "$cmd" >/dev/null 2>&1; then
    echo "ERROR: $cmd is not installed. Install it (e.g. apt install iw iproute2 batctl)" >&2
    exit 1
  fi
done

# Remove mesh interface from batman-adv
if batctl if | grep -q "$MESH_IFACE" 2>/dev/null; then
  batctl if del "$MESH_IFACE" || true
else
  echo "  $MESH_IFACE not currently attached to batman-adv"
fi

# Bring down batman interface
if ip link show "$BAT_IFACE" >/dev/null 2>&1; then
  ip addr flush dev "$BAT_IFACE" || true
  ip link set "$BAT_IFACE" down || true
  # Destroy the batman interface properly
  batctl meshif "$BAT_IFACE" interface destroy 2>/dev/null || true
else
  echo "  $BAT_IFACE does not exist"
fi

# Reset mesh interface and bring it down
if ip link show "$MESH_IFACE" >/dev/null 2>&1; then
  ip link set "$MESH_IFACE" down || true
  
  # Reset interface type back to managed (normal Wi-Fi)
  iw dev "$MESH_IFACE" set type managed || true
  
  ip link set "$MESH_IFACE" up || true
else
  echo "  $MESH_IFACE does not exist"
fi

# Optionally unload batman-adv module
if lsmod | grep -q batman_adv; then
  modprobe -r batman-adv 2>/dev/null || echo "  Could not unload batman-adv (may still be in use)"
fi

# Platform-specific teardown
if [[ "$OFFBOARD_CONTROLLER" == "DESKTOP" ]]; then
  echo "[Desktop Teardown]"
  
  # Restart NetworkManager and wpa_supplicant
  echo "Restarting NetworkManager and wpa_supplicant..."
  systemctl start NetworkManager 2>/dev/null || true
  systemctl start wpa_supplicant 2>/dev/null || true
  
  # Re-enable NetworkManager control of mesh interface
  nmcli dev set "$MESH_IFACE" managed yes 2>/dev/null || true

elif [[ "$OFFBOARD_CONTROLLER" == "PI" ]]; then
  echo "[Raspberry Pi Teardown]"
  
  # Ensure NetworkManager is enabled and running
  systemctl unmask NetworkManager.service 2>/dev/null || true
  systemctl enable --now NetworkManager.service 2>/dev/null || true
  systemctl restart NetworkManager.service 2>/dev/null || true
  
  # Re-enable Wi-Fi radio
  if command -v nmcli >/dev/null 2>&1; then
    nmcli radio wifi on || true
    nmcli device set "$MESH_IFACE" managed yes 2>/dev/null || true
  fi
  
  # Re-enable wpa_supplicant
  systemctl unmask wpa_supplicant.service 2>/dev/null || true
  systemctl enable --now wpa_supplicant.service 2>/dev/null || true
  systemctl restart wpa_supplicant.service 2>/dev/null || true
  
  # Optionally request DHCP lease
  if command -v dhclient >/dev/null 2>&1; then
    dhclient -v "$MESH_IFACE" 2>/dev/null || true
  fi
fi

# Common cleanup
ip route del 224.0.0.0/4 dev "$BAT_IFACE" 2>/dev/null || true
ip route del 239.0.0.0/8 2>/dev/null || true

# Remove ROS2 / DDS configuration files if present
rm -f /etc/profile.d/ros2_batman.sh 2>/dev/null || true
rm -f /etc/fastdds_batman.xml 2>/dev/null || true

echo "Batman-adv mesh teardown complete, you may now reconnect to normal Wi-Fi networks."
