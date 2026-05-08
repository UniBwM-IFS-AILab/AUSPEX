#!/usr/bin/env bash
set -euo pipefail

# get_signal_strength.sh — extract WiFi signal strength
#
# Returns: For managed mode: "AP=XX:XX:XX:XX:XX:XX|sig=-XX"
#          For mesh/IBSS: "MAC=peer1_mac|sig=XX|avg=YY;MAC=peer2_mac|sig=XX|avg=YY;..."
#          Or: "NA" if no connection found

get_all_interfaces() {
  # Find all WiFi interfaces
  iw dev 2>/dev/null | awk '/Interface/ {print $2}'
}

get_managed_mode_signal() {
  local iface="$1"
  
  # For managed mode: extract signal to AP from 'iw link'
  iw dev "$iface" link 2>/dev/null | awk '
    /^Connected/ {
      # Extract AP MAC address (17 chars with colons)
      for (i = 1; i <= NF; i++) {
        if ($i ~ /^[0-9a-fA-F:]+$/ && length($i) == 17) {
          ap_mac = $i
          break
        }
      }
    }
    /signal:/ {
      # Extract signal value
      idx = index($0, "signal:")
      if (idx > 0) {
        part = substr($0, idx + 7)
        gsub(/^[ \t]*/, "", part)
        split(part, vals)
        signal = vals[1]
      }
    }
    END {
      if (signal != "") {
        printf "AP=%s|sig=%s", ap_mac, signal
      }
    }
  '
}

get_mesh_mode_signal() {
  local iface="$1"
  
  # For mesh/IBSS/batman-adv: extract signal from all peers
  # Works even if "signal avg:" is not available (batman-adv compatibility)
  iw dev "$iface" station dump 2>/dev/null | awk '
    /^Station/ { 
      # Output previous entry if we have one
      if (mac != "" && signal != "") {
        if (signal_avg != "") {
          printf "MAC=%s|sig=%s|avg=%s\n", mac, signal, signal_avg
        } else {
          printf "MAC=%s|sig=%s\n", mac, signal
        }
      }
      # Start new entry
      mac = $2
      signal = ""
      signal_avg = ""
    }
    /signal:/ && mac != "" && !/signal avg:/ {
      # Instantaneous signal (not "signal avg:")
      idx = index($0, "signal:")
      if (idx > 0) {
        part = substr($0, idx + 7)
        gsub(/^[ \t]*/, "", part)
        split(part, vals)
        signal = vals[1]
      }
    }
    /signal avg:/ && mac != "" {
      # Average signal
      idx = index($0, "signal avg:")
      if (idx > 0) {
        part = substr($0, idx + 11)
        gsub(/^[ \t]*/, "", part)
        split(part, vals)
        signal_avg = vals[1]
      }
    }
    END {
      # Output last entry if we have one
      if (mac != "" && signal != "") {
        if (signal_avg != "") {
          printf "MAC=%s|sig=%s|avg=%s\n", mac, signal, signal_avg
        } else {
          printf "MAC=%s|sig=%s\n", mac, signal
        }
      }
    }
  ' | paste -sd ';'
}

# Main - check all interfaces and return first valid result
INTERFACES=$(get_all_interfaces)

if [[ -z "$INTERFACES" ]]; then
  echo "NA"
  exit 0
fi

# Try each interface until we find one with signal data
while IFS= read -r IFACE; do
  [[ -z "$IFACE" ]] && continue
  
  # Try managed mode first (most common)
  if iw dev "$IFACE" link 2>/dev/null | grep -q "^Connected"; then
    result=$(get_managed_mode_signal "$IFACE")
    if [[ -n "$result" ]]; then
      echo "$result"
      exit 0
    fi
  fi
  
  # Try mesh/IBSS mode
  result=$(get_mesh_mode_signal "$IFACE")
  if [[ -n "$result" ]]; then
    echo "$result"
    exit 0
  fi
done <<< "$INTERFACES"

# No interface had valid signal data
echo "NA"
