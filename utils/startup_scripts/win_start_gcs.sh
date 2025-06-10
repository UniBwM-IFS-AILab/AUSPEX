#!/bin/bash -i
# bash script for opening multiple terminal windows and starting the ground station modules
exec > /dev/null 2>&1
win_path="/mnt/c/Windows/System32/"
win_path2="C:/Users/%USERNAME%/AppData/Local/Microsoft/WindowsApps"

wsl_profile_name="Dev_Companion22"
wsl_instance_name="Dev_Companion22"

pushd /mnt/c > /dev/null
$win_path/cmd.exe /c "$win_path2/wt.exe --title "Copernicus" -p $wsl_profile_name wsl -d $wsl_instance_name -e bash -li -c \"invasa \"start_copernicus_server\"\" \
    ; new-tab --title "Valkey Server" -p $wsl_profile_name wsl -d $wsl_instance_name -e bash -li -c \"invasa \"start_valkey\"\" \
    ; new-tab --title "KNOW" -p $wsl_profile_name wsl -d $wsl_instance_name -e bash -li -c \"invasa \"start_knowledge_main\"\" \
    ; new-tab --title "UP4ROS2" -p $wsl_profile_name wsl -d $wsl_instance_name -e bash -li -c \"invasa \"start_up4ros\"\" \
    ; new-tab --title "SENS" -p $wsl_profile_name wsl -d $wsl_instance_name -e bash -li -c \"invasa \"start_perception_main\"\" \
    ; new-tab --title "EXEC" -p $wsl_profile_name wsl -d $wsl_instance_name -e bash -li -c \"invasa \"start_executor_main\"\" \
    ; new-tab --title "PLAN" -p $wsl_profile_name wsl -d $wsl_instance_name -e bash -li -c \"invasa \"start_planning_main\"\""
popd > /dev/null
