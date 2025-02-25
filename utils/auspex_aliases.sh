# path to AUSPEX base folder
AUSPEX_HOME="~/AUSPEX"


# Start Drone
alias kill_px4="pkill -x px4 || true"
alias start_px4="cd $AUSPEX_HOME/PX4-Autopilot; make px4_sitl_default none_iris"
alias start_px4_multiple="sleep 1 && $AUSPEX_HOME/PX4-Autopilot/Tools/simulation/sitl_multiple_run.sh"

# Start drone interface
alias start_rtps_agent="export FASTRTPS_DEFAULT_PROFILES_FILE=~/AUSPEX/utils/dds_scripts/fastrtps_dds_tailscale/fastrtps_dds_setup_local.xml && cd $AUSPEX_HOME/Micro-XRCE-DDS-Agent/build; MicroXRCEAgent udp4 -p 8888"
alias start_rtps_agent_drone="export FASTRTPS_DEFAULT_PROFILES_FILE=~/AUSPEX/utils/dds_scripts/fastrtps_dds_tailscale/fastrtps_dds_setup_local.xml && sudo MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600"

# start Copernicus server
alias start_copernicus_server="ros2 run auspex_knowledge copernicus_server"

# start database commands
alias start_valkey="$AUSPEX_HOME/AUSPEX-KNOW/valkey/valkey/src/valkey-server --loadmodule $AUSPEX_HOME/AUSPEX-KNOW/valkey/valkeyJSON/build/src/libjson.so --save "
alias stop_valkey="pkill valkey-server"

# start knowledge main
alias start_knowledge_main="ros2 run auspex_knowledge knowledge_main"

# Start Offboard
alias start_offboard_control="sleep 2; ros2 launch auspex_obc auspex_obc.launch.py"

# start UP interface
alias start_up4ros="ros2 launch up4ros2 up4ros2.launch.py"

# Start Executor
alias start_executor_main="ros2 launch auspex_executor start_executor_main.launch.py"

# Start Planner
alias start_planning_main="sleep 3; ros2 launch auspex_planning start_planning_main.launch.py"

# start perception module
alias start_perception_main="ros2 launch auspex_perception start_perception_main.launch.py"

alias start_command_publisher="ros2 run auspex_planning planning_command_publisher"

# start command for the rosbridge server
alias start_rosbridge="ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
alias start_rosbridge_ip="ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:="

# start command to start the whole simulation
alias start_simulation="$AUSPEX_HOME/utils/startup_scripts/start_simulation.sh"
alias start_hitl="$AUSPEX_HOME/utils/startup_scripts/start_hitl.sh"

alias test_takeoff="ros2 action send_goal /vhcl0/takeoff auspex_msgs/action/Takeoff '{takeoff_height: 300}'"
alias test_land="ros2 action send_goal /vhcl0/land auspex_msgs/action/Land '{land_speed: 1}'"

alias edit_alias="sudo nano ~/auspex_aliases.sh"
alias reload_alias="source ~/auspex_aliases.sh"

alias edit_exports="sudo nano ~/auspex_exports.sh"
alias reload_exports="source ~/auspex_exports.sh"

alias source_auspex_exec="source $AUSPEX_HOME/AUSPEX-EXEC/install/setup.bash"
alias source_auspex_plan="source $AUSPEX_HOME/AUSPEX-PLAN/install/setup.bash"
alias source_auspex_sens="source $AUSPEX_HOME/AUSPEX-SENS/install/setup.bash"
alias source_auspex_aero="source $AUSPEX_HOME/AUSPEX-AERO/install/setup.bash"
alias source_auspex_msgs="source $AUSPEX_HOME/AUSPEX-MSGS/install/setup.bash"
alias source_auspex_know="source $AUSPEX_HOME/AUSPEX-KNOW/install/setup.bash"

# source menthon
alias source_menthon_ws="source $HOME/MENTHON-WS/install/setup.bash"

alias source_all="source_auspex_exec && source_auspex_plan && source_auspex_sens && source_auspex_msgs && source_auspex_aero && source_auspex_know"

alias build_exec="cd $AUSPEX_HOME/AUSPEX-EXEC && colcon build"
alias build_plan="cd $AUSPEX_HOME/AUSPEX-PLAN && colcon build"
alias build_sens="cd $AUSPEX_HOME/AUSPEX-SENS && colcon build"
alias build_msgs="cd $AUSPEX_HOME/AUSPEX-MSGS && colcon build"
alias build_aero="cd $AUSPEX_HOME/AUSPEX-AERO && colcon build"
alias build_know="cd $AUSPEX_HOME/AUSPEX-KNOW && colcon build"

alias rebuild_auspex="$AUSPEX_HOME/utils/dev_scripts/rebuild_auspex.sh"
alias clean_auspex="$AUSPEX_HOME/utils/dev_scripts/clean_auspex.sh"
alias pull_auspex="$AUSPEX_HOME/utils/dev_scripts/pull_auspex.sh"

alias eb='sudo nano ~/.bashrc'
alias sb='source ~/.bashrc'

alias aero='cd ~/AUSPEX/AUSPEX-AERO/'
alias sens='cd ~/AUSPEX/AUSPEX-SENS/'
alias msgs='cd ~/AUSPEX/AUSPEX-MSGS/'
alias exec='cd ~/AUSPEX/AUSPEX-EXEC'
alias plan='cd ~/AUSPEX/AUSPEX-PLAN'
alias know='cd ~/AUSPEX/AUSPEX-KNOW'
alias auspex='cd ~/AUSPEX'

# scripts 
alias install_auspex="cd $AUSPEX_HOME/ && ./install.sh TERRA"
alias set_msg_ctx="$AUSPEX_HOME/utils/dev_scripts/set_msg_context.sh"
alias change_rmw='~/AUSPEX/utils/dds_scripts/fastrtps_dds_tailscale/update_fastdds_xml.sh '
