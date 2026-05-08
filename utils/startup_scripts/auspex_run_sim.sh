#!/bin/bash

set -euo pipefail

MODE="${1:-}"
PLATFORM_COUNT="${2:-1}"

usage() {
    echo "Usage: $0 {ue|is|gcs} [platform_count]"
    echo "  ue  : Start simulation with PX4 + AERO + full stack"
    echo "  is  : Start simulation with AERO + full stack"
    echo "  gcs : Start ground control environment"
}

is_wsl() {
    [[ -n "${WSL_INTEROP:-}" ]] || grep -qi "microsoft" /proc/version
}

if [[ "$MODE" != "ue" && "$MODE" != "is" && "$MODE" != "gcs" ]]; then
    usage
    exit 1
fi

if [[ "$MODE" == "ue" || "$MODE" == "is" ]]; then
    if ! [[ "$PLATFORM_COUNT" =~ ^[1-9][0-9]*$ ]]; then
        echo "Error: platform_count must be a positive integer."
        exit 1
    fi
fi

if [[ "$MODE" == "ue" ]]; then
    if is_wsl; then
        "$HOME/AUSPEX/utils/startup_scripts/auspex_run_win_ue_helper.sh" "$PLATFORM_COUNT"
    else
        gnome-terminal --window --title="PX4" -- bash -c "$HOME/AUSPEX/utils/startup_scripts/auspex_run_lin_ue_helper.sh $PLATFORM_COUNT" &
    fi
fi

if [[ "$MODE" == "is" ]]; then
    if is_wsl; then
        echo "Not implemented: Starting IS simulation in WSL is not currently supported."
    else
        gnome-terminal --window --title="AERO" -- bash -c "$HOME/AUSPEX/utils/startup_scripts/auspex_run_lin_is_helper.sh $PLATFORM_COUNT" &
    fi
    echo "Started IS simulation with platform_count=$PLATFORM_COUNT"
fi

if [[ "$MODE" == "gcs" ]]; then
    if is_wsl; then
        "$HOME/AUSPEX/utils/startup_scripts/auspex_run_win_gcs_helper.sh"
    else
        gnome-terminal --window --title="Height Server" -- bash -i -c "$HOME/AUSPEX/utils/startup_scripts/auspex_run_lin_gcs_helper.sh" &
        echo "Ground station window with tabs started."
    fi
fi