#!/bin/bash

set -euo pipefail

DRONE_NAME="${1:-}"
TAB_COUNT="${2:-2}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AUSPEX_HOME="$(cd "${SCRIPT_DIR}/../.." && pwd)"
LOOKUP_FILE="${AUSPEX_HOME}/utils/ovpn/ip_lookup.txt"
AUTH_FILE="${AUSPEX_HOME}/utils/ovpn/auth.txt"
LIN_HELPER_SCRIPT="${SCRIPT_DIR}/auspex_ssh_connect_lin_helper.sh"
WIN_HELPER_SCRIPT="${SCRIPT_DIR}/auspex_ssh_connect_win_helper.sh"
SSH_PASSWORD=""

usage() {
    echo "Usage: $0 <drone_name> [tab_count]"
    echo "Example: $0 hummingbird0 3"
}

is_wsl() {
    [[ -n "${WSL_INTEROP:-}" ]] || grep -qi "microsoft" /proc/version
}

if [[ -z "$DRONE_NAME" ]]; then
    usage
    exit 1
fi

if ! [[ "$TAB_COUNT" =~ ^[1-9][0-9]*$ ]]; then
    echo "Error: tab_count must be a positive integer."
    usage
    exit 1
fi

if [[ ! -f "$LOOKUP_FILE" ]]; then
    echo "Error: lookup file not found at $LOOKUP_FILE"
    exit 1
fi

if [[ ! -f "$AUTH_FILE" ]]; then
    echo "Error: auth file not found at $AUTH_FILE"
    exit 1
fi

if ! command -v sshpass >/dev/null 2>&1; then
    echo "Error: sshpass is required but not installed."
    echo "Install with: sudo apt install sshpass"
    exit 1
fi

SSH_PASSWORD="$(sed -n '2p' "$AUTH_FILE" | tr -d '\r')"
if [[ -z "$SSH_PASSWORD" ]]; then
    echo "Error: SSH password not found on line 2 of $AUTH_FILE"
    exit 1
fi

TARGET_IP="$(awk -F',' -v key="$DRONE_NAME" '$1 == key {print $2; exit}' "$LOOKUP_FILE")"
if [[ -z "$TARGET_IP" ]]; then
    echo "Error: no IP found for '$DRONE_NAME' in $LOOKUP_FILE"
    exit 1
fi

SSH_TARGET="${DRONE_NAME}@${TARGET_IP}"

if is_wsl; then
    if ! command -v cmd.exe >/dev/null 2>&1; then
        echo "Error: cmd.exe is required but not accessible (are you in WSL?)"
        exit 1
    fi
    if [[ ! -f "$WIN_HELPER_SCRIPT" ]]; then
        echo "Error: helper script not found at $WIN_HELPER_SCRIPT"
        exit 1
    fi
    "${WIN_HELPER_SCRIPT}" "$DRONE_NAME" "$SSH_PASSWORD" "$TARGET_IP" "$TAB_COUNT"
else
    if ! command -v gnome-terminal >/dev/null 2>&1; then
        echo "Error: gnome-terminal is required but not installed."
        exit 1
    fi
    if [[ ! -f "$LIN_HELPER_SCRIPT" ]]; then
        echo "Error: helper script not found at $LIN_HELPER_SCRIPT"
        exit 1
    fi
    gnome-terminal --window --title="${DRONE_NAME} SSH 1" -- \
        bash -c "bash \"${LIN_HELPER_SCRIPT}\" \"${DRONE_NAME}\" \"${SSH_PASSWORD}\" \"${TARGET_IP}\" \"${TAB_COUNT}\"" &
    echo "Opened one terminal window with ${TAB_COUNT} SSH tab(s) to ${SSH_TARGET}."
fi