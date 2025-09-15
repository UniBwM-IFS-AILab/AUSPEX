#!/bin/bash
# Helper script that runs inside the new terminal and adds tabs to itself for GCS

# Wait a moment to ensure the terminal is fully initialized
sleep 1

# Add tabs to the current terminal window
gnome-terminal --tab --title="Valkey" -- bash -i -c "iv run_valkey"
gnome-terminal --tab --title="KNOW" -- bash -i -c "iv run_know"
gnome-terminal --tab --title="SENS" -- bash -i -c "iv run_sens"
gnome-terminal --tab --title="EXEC" -- bash -i -c "iv run_exec"
gnome-terminal --tab --title="PLAN" -- bash -i -c "iv run_plan"

bash -i -c "iv run_copernicus"