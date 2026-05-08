#!/bin/bash
# Helper script that runs inside the new terminal and adds tabs to itself

if [ -z "$1" ]; then
    echo "No drone count provided. Starting with platform_count = 1."
    platform_count=5
else
    platform_count=$1
fi

# Wait a moment to ensure the terminal is fully initialized
sleep 1

# Add tabs to the current terminal window
gnome-terminal --tab --title="Height Server" -- bash -i -c "vasa_cmd height_server_run"
gnome-terminal --tab --title="Valkey" -- bash -i -c "vasa_cmd valkey_run"
gnome-terminal --tab --title="KNOW" -- bash -i -c "vasa_cmd know_run"
gnome-terminal --tab --title="SENS" -- bash -i -c "vasa_cmd sens_run"
gnome-terminal --tab --title="EXEC" -- bash -i -c "vasa_cmd \"exec_run count:=$platform_count\""
gnome-terminal --tab --title="CTRL" -- bash -i -c "vasa_cmd ctrl_run"
gnome-terminal --tab --title="PLAN" -- bash -i -c "vasa_cmd plan_run"

# Run AERO in the main terminal using interactive bash
bash -i -c "vasa_cmd \"aero_run count:=$platform_count\""