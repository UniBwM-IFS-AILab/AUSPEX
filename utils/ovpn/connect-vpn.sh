#!/bin/bash

sudo pkill -SIGINT openvpn

OVPN_DIR="$HOME/AUSPEX/utils/ovpn"

echo $OVPN_DIR

CONFIG_FILE="$OVPN_DIR/client_config.ovpn"

AUTH_FILE="$OVPN_DIR/auth.txt"

sudo openvpn --config "$CONFIG_FILE" --auth-user-pass "$AUTH_FILE" &
