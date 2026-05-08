#!/bin/bash -i
# bash script for opening multiple terminal windows and starting the ground station modules
exec > /dev/null 2>&1
win_path="/mnt/c/Windows/System32/"
win_path2="C:/Users/%USERNAME%/AppData/Local/Microsoft/WindowsApps"

wsl_profile_name="Dev_Companion22"
wsl_instance_name="Dev_Companion22"

pushd /mnt/c > /dev/null
$win_path/cmd.exe /c "$win_path2/wt.exe --title "Height Server" -p $wsl_profile_name wsl -d $wsl_instance_name -e bash -li -c \"vasa_cmd \"height_server_run\"\" \
    ; new-tab --title "Valkey" -p $wsl_profile_name wsl -d $wsl_instance_name -e bash -li -c \"vasa_cmd \"valkey_run\"\" \
    ; new-tab --title "KNOW" -p $wsl_profile_name wsl -d $wsl_instance_name -e bash -li -c \"vasa_cmd \"know_run\"\" \
    ; new-tab --title "SENS" -p $wsl_profile_name wsl -d $wsl_instance_name -e bash -li -c \"vasa_cmd \"sens_run\"\" \
    ; new-tab --title "CTRL" -p $wsl_profile_name wsl -d $wsl_instance_name -e bash -li -c \"vasa_cmd \"ctrl_run\"\" \
    ; new-tab --title "PLAN" -p $wsl_profile_name wsl -d $wsl_instance_name -e bash -li -c \"vasa_cmd \"plan_run\"\""
popd > /dev/null
