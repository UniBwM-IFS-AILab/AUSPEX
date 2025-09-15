#!/bin/bash
# Script to start simulation pipeline in multiple gnome-terminal tabs on Ubuntu 24

if [ -z "$1" ]; then
    echo "No drone count provided. Starting with drone_count = 1."
    drone_count=1
else
    drone_count=$1
fi

# Start a new terminal window and run the helper script in it
gnome-terminal --window --title="PX4" -- bash -c "/home/doeschbj/AUSPEX/utils/startup_scripts/ue_lin_start_sim_helper.sh $drone_count" &

echo "Simulation window with tabs started."
