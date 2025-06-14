#!/bin/bash -i
# bash script for opening multiple terminal windows and starting the simulation pipeline


if [ -z "$1" ]; then
    echo "no drone count provided. starting with drone_count = 1."
    drone_count=1
else
    drone_count=$1
fi

exec > /dev/null 2>&1

win_path="/mnt/c/Windows/System32/"
win_path2="C:/Users/%USERNAME%/AppData/Local/Microsoft/WindowsApps"

wsl_profile_name="Dev_Companion22"
wsl_instance_name="Dev_Companion22"
pushd /mnt/c > /dev/null
$win_path/cmd.exe /c "$win_path2/wt.exe --title "uXRCE" -p $wsl_profile_name wsl -d $wsl_instance_name -e bash -li -c \"invasa \\\"start_rtps_agent\\\"\"\
    ; new-tab --title "PX4" -p $wsl_profile_name wsl -d $wsl_instance_name -e bash -li -c \"invasa \\\"start_px4_multiple ${drone_count}\\\"\" \
    ; new-tab --title "Copernicus" -p $wsl_profile_name wsl -d $wsl_instance_name -e bash -li -c \"invasa \"start_copernicus_server\"\" \
    ; new-tab --title "Valkey Server" -p $wsl_profile_name wsl -d $wsl_instance_name -e bash -li -c \"invasa \"start_valkey\"\" \
    ; new-tab --title "KNOW" -p $wsl_profile_name wsl -d $wsl_instance_name -e bash -li -c \"invasa \"start_knowledge_main\"\" \
    ; new-tab --title "AERO" -p $wsl_profile_name wsl -d $wsl_instance_name -e bash -li -c \"invasa \\\"start_offboard_control count:=${drone_count}\\\"\" \
    ; new-tab --title "SENS" -p $wsl_profile_name wsl -d $wsl_instance_name -e bash -li -c \"invasa \"start_perception_main\"\" \
    ; new-tab --title "EXEC" -p $wsl_profile_name wsl -d $wsl_instance_name -e bash -li -c \"invasa \"start_executor_main\"\" \
    ; new-tab --title "PLAN" -p $wsl_profile_name wsl -d $wsl_instance_name -e bash -li -c \"invasa start_planning_main\""
popd > /dev/null