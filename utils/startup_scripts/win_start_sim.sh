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
$win_path/cmd.exe /c "$win_path2/wt.exe --title "PX4" -p $wsl_profile_name wsl -d $wsl_instance_name -e bash -li -c \"invasa \\\"run_px4 ${drone_count}\\\"\"\
    ; new-tab --title "AERO" -p $wsl_profile_name wsl -d $wsl_instance_name -e bash -li -c \"invasa \\\"run_aero count:=${drone_count}\\\"\" \
    ; new-tab --title "Copernicus" -p $wsl_profile_name wsl -d $wsl_instance_name -e bash -li -c \"invasa \"run_copernicus\"\" \
    ; new-tab --title "Valkey Server" -p $wsl_profile_name wsl -d $wsl_instance_name -e bash -li -c \"invasa \"run_valkey\"\" \
    ; new-tab --title "KNOW" -p $wsl_profile_name wsl -d $wsl_instance_name -e bash -li -c \"invasa \"run_know\"\" \
    ; new-tab --title "SENS" -p $wsl_profile_name wsl -d $wsl_instance_name -e bash -li -c \"invasa \"run_sens\"\" \
    ; new-tab --title "EXEC" -p $wsl_profile_name wsl -d $wsl_instance_name -e bash -li -c \"invasa \"run_exec\"\" \
    ; new-tab --title "PLAN" -p $wsl_profile_name wsl -d $wsl_instance_name -e bash -li -c \"invasa \"run_plan\"\""
popd > /dev/null