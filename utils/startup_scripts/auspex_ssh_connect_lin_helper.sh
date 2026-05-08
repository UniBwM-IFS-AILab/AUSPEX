#!/bin/bash
# Helper script that runs inside the new terminal and adds a second SSH tab

set -euo pipefail

DRONE_NAME="${1:-}"
SSH_PASSWORD="${2:-}"
TARGET_IP="${3:-}"
TAB_COUNT="${4:-2}"

if [[ -z "$DRONE_NAME" || -z "$SSH_PASSWORD" || -z "$TARGET_IP" ]]; then
    echo "Usage: $0 <drone_name> <ssh_password> <target_ip> [tab_count]"
    exit 1
fi

if ! [[ "$TAB_COUNT" =~ ^[1-9][0-9]*$ ]]; then
    echo "Error: tab_count must be a positive integer."
    exit 1
fi

SSH_TARGET="${DRONE_NAME}@${TARGET_IP}"
SSH_CMD="sshpass -p '${SSH_PASSWORD}' ssh -o StrictHostKeyChecking=accept-new ${SSH_TARGET}"

# Give the newly opened terminal window a moment to initialize before adding tabs.
sleep 1

for ((i = 2; i <= TAB_COUNT; i++)); do
    gnome-terminal --tab --title="${DRONE_NAME} SSH ${i}" -- bash -i -c "${SSH_CMD}; exec bash"
done

# Run the first SSH session in the original tab.
bash -i -c "${SSH_CMD}; exec bash"
