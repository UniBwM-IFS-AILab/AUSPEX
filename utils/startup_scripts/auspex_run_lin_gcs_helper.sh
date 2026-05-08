#!/bin/bash
# Helper script that runs inside the new terminal and adds tabs to itself for GCS

# Wait a moment to ensure the terminal is fully 
sleep 1

# Start Valkey first to handle issues with know not finding valkey
gnome-terminal --tab --title="Valkey" -- bash -i -c "vasa_cmd valkey_run"
sleep 1

# Add tabs to the current terminal window
gnome-terminal --tab --title="KNOW" -- bash -i -c "vasa_cmd know_run"
gnome-terminal --tab --title="SENS" -- bash -i -c "vasa_cmd sens_run"
gnome-terminal --tab --title="CTRL" -- bash -i -c "vasa_cmd ctrl_run"
gnome-terminal --tab --title="PLAN" -- bash -i -c "vasa_cmd plan_run"

bash -i -c "vasa_cmd height_server_run"