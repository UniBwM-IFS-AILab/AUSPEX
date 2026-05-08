#!/usr/bin/env bash
set -euo pipefail

# ============================================================================
# Batman-adv mesh network setup script
# Unified script for Desktop Linux and Raspberry Pi
# Reads configuration from platform_properties.json
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

# Batman-adv specific configuration
# These should be defined in the JSON file under a "network" section
PLATFORM_IP=$(jq -r '.network.ip' "$CONFIG_FILE")
MESH_IFACE=$(jq -r '.network.mesh_interface' "$CONFIG_FILE")
MESH_NAME=$(jq -r '.network.mesh_name' "$CONFIG_FILE")
MESH_FREQ=$(jq -r '.network.mesh_frequency' "$CONFIG_FILE")
BAT_IFACE=$(jq -r '.network.batman_interface' "$CONFIG_FILE")
MESH_MODE=$(jq -r '.network.mesh_interface_mode' "$CONFIG_FILE")
MESH_MODE_LOWER=$(echo "$MESH_MODE" | tr '[:upper:]' '[:lower:]')
MESH_BSSID=$(jq -r '.network.mesh_BSSID' "$CONFIG_FILE")

# Read offboard_controller from platform details
OFFBOARD_CONTROLLER=$(jq -r '.platform_details.offboard_controller' "$CONFIG_FILE")

# Construct IP address with CIDR notation if not present
if [[ "$PLATFORM_IP" != *"/"* ]]; then
  BAT_IP="${PLATFORM_IP}/24"
else
  BAT_IP="$PLATFORM_IP"
fi

echo "Platform ID:        $PLATFORM_ID"
echo "Platform IP:        $BAT_IP"
echo "Mesh Interface:     $MESH_IFACE"
echo "Mesh Name:          $MESH_NAME"
echo "Mesh Frequency:     $MESH_FREQ MHz"
echo "Mesh Interface Mode: $MESH_MODE"
echo "Mesh BSSID:         $MESH_BSSID"
echo "Batman Interface:   $BAT_IFACE"
echo "Offboard Controller: $OFFBOARD_CONTROLLER"
echo

echo "Running batman-adv mesh setup..."

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
for cmd in iw ip batctl modprobe rfkill; do
  if ! command -v "$cmd" >/dev/null 2>&1; then
    echo "ERROR: $cmd is not installed. Install it (e.g. apt install iw iproute2 batctl rfkill)" >&2
    exit 1
  fi
done

# Platform-specific setup
if [[ "$OFFBOARD_CONTROLLER" == "DESKTOP" ]]; then
  echo "[Desktop Setup]"
  
  # Check for nmcli
  if ! command -v nmcli >/dev/null 2>&1; then
    echo "ERROR: nmcli is not installed. Install it (e.g. apt install network-manager)" >&2
    exit 1
  fi
  
  # Desktop-specific setup - disable NetworkManager management for the mesh interface only
  # This allows other WiFi adapters and ethernet to remain active
  echo "Disabling NetworkManager management for $MESH_IFACE only..."
  echo "Other network interfaces will remain active."
  
  # Set the mesh interface to unmanaged by NetworkManager
  nmcli device set "$MESH_IFACE" managed no 2>/dev/null || true
  
  # Stop any wpa_supplicant instance on this specific interface
  pkill -f "wpa_supplicant.*$MESH_IFACE" 2>/dev/null || true
  
  # Unblock WiFi (if needed)
  rfkill unblock wifi || true
  sleep 2

elif [[ "$OFFBOARD_CONTROLLER" == "PI" ]]; then
  echo "[Raspberry Pi Setup]"
  
  # Check for iwconfig
  if ! command -v iwconfig >/dev/null 2>&1; then
    echo "ERROR: iwconfig is not installed. Install it (e.g. apt install wireless-tools)" >&2
    exit 1
  fi
  
  # Stop any wpa_supplicant instance on this specific interface
  pkill -f "wpa_supplicant.*$MESH_IFACE" 2>/dev/null || true
  
  # Unblock WiFi
  rfkill unblock wifi || true
  sleep 2
fi

# Common batman-adv setup
# Load batman-adv kernel module
if ! lsmod | grep -q batman_adv; then
  modprobe batman-adv
  sleep 0.5
fi

# Reset Wi-Fi interface
if ! ip link show "$MESH_IFACE" >/dev/null 2>&1; then
  echo "ERROR: interface $MESH_IFACE not found. Aborting." >&2
  ip -br link show
  exit 1
fi

ip link set "$MESH_IFACE" down

# Check driver support
if ! iw list | grep -A5 "Supported interface modes" | grep -qi "$MESH_MODE"; then
  echo "WARNING: Driver may not support $MESH_MODE mode" >&2
fi

# Platform-specific IBSS setup
if [[ "$OFFBOARD_CONTROLLER" == "DESKTOP" ]]; then
  # Desktop: Use iw commands
  echo "Configuring IBSS network with iw..."
  echo "  Network: $MESH_NAME"
  echo "  BSSID:   $MESH_BSSID"
  echo "  Freq:    $MESH_FREQ MHz"
  
  # Set interface type (while interface is DOWN)
  echo "Setting interface to $MESH_MODE mode..."
  set +e
  iw dev "$MESH_IFACE" set type "$MESH_MODE_LOWER"
  rc=$?
  set -e
  if [ "$rc" -ne 0 ]; then
    echo "ERROR: 'iw set type $MESH_MODE_LOWER' failed (rc=$rc). See dmesg for details." >&2
    dmesg | tail -n 40
    exit 1
  fi
  
  # Bring up interface
  ip link set "$MESH_IFACE" up
  sleep 0.5
  
  # Force leave any auto-created IBSS
  iw dev "$MESH_IFACE" "$MESH_MODE_LOWER" leave 2>/dev/null || true
  sleep 0.5
  
  # Join with our parameters - try with HT20 first, fallback without it
  set +e
  iw dev "$MESH_IFACE" "$MESH_MODE_LOWER" join "$MESH_NAME" "$MESH_FREQ" HT20 fixed-freq "$MESH_BSSID" 2>/dev/null
  rc=$?
  if [ "$rc" -ne 0 ]; then
    echo "  HT20 parameter not supported, trying without it..."
    iw dev "$MESH_IFACE" "$MESH_MODE_LOWER" join "$MESH_NAME" "$MESH_FREQ" fixed-freq "$MESH_BSSID" 2>/dev/null
    rc=$?
    if [ "$rc" -ne 0 ]; then
      echo "  fixed-freq parameter not supported, trying basic join..."
      iw dev "$MESH_IFACE" "$MESH_MODE_LOWER" join "$MESH_NAME" "$MESH_FREQ" "$MESH_BSSID"
      rc=$?
    fi
  fi
  set -e
  
  if [ "$rc" -ne 0 ]; then
    echo "ERROR: Failed to join IBSS (rc=$rc)." >&2
    echo "Recent kernel messages:" >&2
    dmesg | tail -n 40
    exit 1
  fi
  
  sleep 2
  echo "Verifying IBSS connection..."
  iw dev "$MESH_IFACE" info
  
elif [[ "$OFFBOARD_CONTROLLER" == "PI" ]]; then
  # Raspberry Pi: Use iwconfig commands
  # Function to convert frequency (MHz) to channel number
  freq_to_channel() {
    local freq=$1
    if [ "$freq" -ge 2412 ] && [ "$freq" -le 2484 ]; then
      # 2.4 GHz band
      if [ "$freq" -eq 2484 ]; then
        echo 14
      else
        echo $(( (freq - 2407) / 5 ))
      fi
    elif [ "$freq" -ge 5170 ] && [ "$freq" -le 5825 ]; then
      # 5 GHz band
      echo $(( (freq - 5000) / 5 ))
    else
      echo "ERROR: Unsupported frequency: $freq MHz" >&2
      exit 1
    fi
  }
  
  MESH_CHANNEL=$(freq_to_channel "$MESH_FREQ")
  
  # echo "Configuring IBSS network with iwconfig..."
  # echo "  Network: $MESH_NAME"
  # echo "  BSSID:   $MESH_BSSID"
  # echo "  Channel: $MESH_CHANNEL (Freq: $MESH_FREQ MHz)"
  
  # Set interface to ad-hoc mode while down
  iwconfig "$MESH_IFACE" mode ad-hoc || {
    echo "ERROR: Failed to set ad-hoc mode" >&2
    exit 1
  }
  
  # Bring interface up
  ip link set "$MESH_IFACE" up
  sleep 1
  
  # Configure ESSID (network name)
  iwconfig "$MESH_IFACE" essid "$MESH_NAME" || {
    echo "ERROR: Failed to set ESSID" >&2
    exit 1
  }
  
  # Set BSSID (cell ID)
  iwconfig "$MESH_IFACE" ap "$MESH_BSSID" || {
    echo "ERROR: Failed to set BSSID" >&2
    exit 1
  }
  
  # Set channel
  iwconfig "$MESH_IFACE" channel "$MESH_CHANNEL" || {
    echo "ERROR: Failed to set channel" >&2
    exit 1
  }
  
  # Wait for connection to establish
  sleep 2
  
  # Verify connection
  # echo "Verifying IBSS connection..."
  # iwconfig "$MESH_IFACE" | grep -E "Mode:|Cell:|ESSID:|Frequency:" || true
fi

# Attach $MESH_IFACE to batman-adv
set +e
batctl if add "$MESH_IFACE"
rc=$?
set -e
if [ "$rc" -ne 0 ]; then
  echo "batctl if add returned exit code $rc. Showing dmesg:" >&2
  dmesg | tail -n 50
fi

# Ensure bat interface exists
if ! ip link show "$BAT_IFACE" >/dev/null 2>&1; then
  set +e
  ip link add name "$BAT_IFACE" type batadv
  rc=$?
  set -e
  if [ "$rc" -ne 0 ]; then
    echo "WARNING: could not create $BAT_IFACE explicitly (rc=$rc). Continuing to attempt to bring it up." >&2
  fi
fi

ip link set "$BAT_IFACE" up || { 
  echo "ERROR: cannot bring $BAT_IFACE up"
  ip link show
  dmesg | tail -n 50
  exit 1
}

ip addr flush dev "$BAT_IFACE" || true
ip addr add "$BAT_IP" dev "$BAT_IFACE" || { 
  echo "ERROR: ip addr add failed"
  ip addr show dev "$BAT_IFACE"
  dmesg | tail -n 50
  exit 1
}

# Ensure IPv4 multicast routes via $BAT_IFACE (ROS2/DDS discovery)
ip route replace 224.0.0.0/4 dev "$BAT_IFACE"

sleep 2

# Success message
echo "Batman-adv mesh setup complete for platform: $PLATFORM_ID with IP: $BAT_IP"
echo ""
echo "Final verification:"
iw dev "$MESH_IFACE" link || echo "Warning: Could not verify IBSS link status"

# Verify configured MESH_BSSID matches the actual IBSS BSSID
echo ""
echo "Verifying configured BSSID..."
ACTUAL_BSSID=$(iw dev "$MESH_IFACE" link 2>/dev/null | grep -oP 'Joined IBSS \K[0-9a-f:]+' || echo "")

if [ -z "$ACTUAL_BSSID" ]; then
  echo "WARNING: Could not extract BSSID from IBSS link status" >&2
else
  ACTUAL_BSSID_UPPER=$(echo "$ACTUAL_BSSID" | tr '[:lower:]' '[:upper:]')
  MESH_BSSID_UPPER=$(echo "$MESH_BSSID" | tr '[:lower:]' '[:upper:]')
  
  if [ "$ACTUAL_BSSID_UPPER" = "$MESH_BSSID_UPPER" ]; then
    echo "✓ Configured BSSID matches IBSS BSSID: $ACTUAL_BSSID"
  else
    echo "✗ BSSID mismatch!"
    echo "  Configured: $MESH_BSSID"
    echo "  Actual:     $ACTUAL_BSSID"
    exit 1
  fi
fi
