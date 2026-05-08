#!/bin/bash

set -euo pipefail

# With `set -e`, pkill exits 1 when no process matches; guard to avoid aborting.
if pgrep -x openvpn >/dev/null 2>&1; then
	sudo pkill -SIGINT openvpn
fi

OVPN_DIR="$HOME/AUSPEX/utils/ovpn"

echo $OVPN_DIR

CONFIG_FILE="$OVPN_DIR/client_config.ovpn"

AUTH_FILE="$OVPN_DIR/auth.txt"

VPN_IFACE="tap0"

sudo openvpn --config "$CONFIG_FILE" --auth-user-pass "$AUTH_FILE" &

# Wait until VPN interface exists and has an IPv4 address
for _ in $(seq 1 30); do
	if ip -4 addr show dev "$VPN_IFACE" 2>/dev/null | grep -q "inet "; then
		break
	fi
	sleep 1
done

if ! ip -4 addr show dev "$VPN_IFACE" 2>/dev/null | grep -q "inet "; then
	echo "ERROR: $VPN_IFACE did not get an IPv4 address within timeout" >&2
	exit 1
fi

# Ensure IPv4 multicast discovery traffic uses VPN interface (ROS2/DDS)
sudo ip route replace 224.0.0.0/4 dev "$VPN_IFACE"
sudo ip route replace 239.255.0.0/16 dev "$VPN_IFACE"

echo "Configured multicast routes via $VPN_IFACE"
ip -4 addr show dev "$VPN_IFACE"
ip route get 239.255.0.1
