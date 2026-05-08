#!/bin/bash -i
# Helper script that opens Windows Terminal tabs for SSH connections

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
WSL_DISTRO="${WSL_DISTRO_NAME:-}"

if [[ -z "$WSL_DISTRO" ]]; then
    echo "Error: WSL_DISTRO_NAME is not set."
    exit 1
fi

SSH_PASSWORD_Q="$(printf '%q' "$SSH_PASSWORD")"
SSH_TARGET_Q="$(printf '%q' "$SSH_TARGET")"
TAB_BASH_CMD="sshpass -p ${SSH_PASSWORD_Q} ssh -o StrictHostKeyChecking=accept-new ${SSH_TARGET_Q}"

exec > /dev/null 2>&1

win_path="/mnt/c/Windows/System32/"
win_path2="C:/Users/%USERNAME%/AppData/Local/Microsoft/WindowsApps"

wsl_profile_name="Dev_Companion22"
wsl_instance_name="Dev_Companion22"

wt_cmd="${win_path2}/wt.exe --title \"SSH1\" -p ${wsl_profile_name} wsl -d ${wsl_instance_name} -e bash -li -c \"${TAB_BASH_CMD}\""
for ((i = 2; i <= TAB_COUNT; i++)); do
    wt_cmd+=" ; new-tab --title \"SSH${i}\" -p ${wsl_profile_name} wsl -d ${wsl_instance_name} -e bash -li -c \"${TAB_BASH_CMD}\""
done

pushd /mnt/c > /dev/null
"${win_path}/cmd.exe" /c "$wt_cmd"
popd > /dev/null