#!/bin/bash
# Helper script that runs inside the new terminal and adds tabs to itself

if [ -z "$1" ]; then
    echo "No drone count provided. Starting with drone_count = 1."
    drone_count=5
else
    drone_count=$1
fi

# Wait a moment to ensure the terminal is fully initialized
sleep 1

# Add tabs to the current terminal window
gnome-terminal --tab --title="Copernicus" -- bash -i -c "iv run_copernicus"
gnome-terminal --tab --title="Valkey Server" -- bash -i -c "iv run_valkey"
gnome-terminal --tab --title="KNOW" -- bash -i -c "iv run_know"
gnome-terminal --tab --title="SENS" -- bash -i -c "iv run_sens"
gnome-terminal --tab --title="EXEC" -- bash -i -c "iv run_exec"
gnome-terminal --tab --title="PLAN" -- bash -i -c "iv run_plan"

# Run AERO in the main terminal using interactive bash
bash -i -c "iv \"run_aero count:=$drone_count\""