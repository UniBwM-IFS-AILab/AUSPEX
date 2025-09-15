#!/bin/bash
# Script to start ground station modules in multiple gnome-terminal tabs on Ubuntu 24

# Start a new terminal window and run the helper script in it
gnome-terminal --window --title="Copernicus" -- bash -i -c "/home/doeschbj/AUSPEX/utils/startup_scripts/lin_start_gcs_helper.sh" &

echo "Ground station window with tabs started."
