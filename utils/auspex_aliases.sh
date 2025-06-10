# path to AUSPEX base folder
AUSPEX_HOME="/root/AUSPEX"

# start/stop PX4 autopilot
alias kill_px4="pkill -x px4 || true"
alias start_px4_multiple="/root/scripts/sitl_multiple_run.sh"

# start uxrce agent
alias start_rtps_agent="MicroXRCEAgent udp4 -p 8888"
alias start_rtps_agent_pi="MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600 -r ~/dds_local_setup.xml"
alias start_rtps_agent_jetson='MicroXRCEAgent serial --dev /dev/ttyTHS0 -b 921600 -r ~/dds_local_setup.xml'
alias start_avis_dds='fastdds discovery -i 1 -l 127.0.0.1 -p 11811'

# start copernicus server
alias start_copernicus_server="ros2 run auspex_knowledge copernicus_server"

# start/stop valkey
alias start_valkey="$AUSPEX_HOME/utils/valkey_conf/start_valkey.sh"
alias stop_valkey="pkill valkey-server"

# start AUSPEX_KNOW
alias start_knowledge_main="sleep 2; ros2 run auspex_knowledge knowledge_main"

# start AUSPEX_AERO
alias start_offboard_control="sleep 3; ros2 launch auspex_obc auspex_obc.launch.py"

# start UP4ROS
alias start_up4ros="ros2 launch upf4ros2 upf4ros2.launch.py"

# start AUSPEX_EXEC
alias start_executor_main="ros2 launch auspex_executor start_executor_main.launch.py"

# start AUSPEX_PLAN
alias start_planning_main="sleep 3; ros2 launch auspex_planning start_planning_main.launch.py"

# start AUSPEX_SENS
alias start_perception_main="ros2 launch auspex_perception start_perception_main.launch.py"

# start command publisher
alias start_command_publisher="ros2 run auspex_planning planning_command_publisher"

# start rosbridge server
alias start_rosbridge="ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
alias start_rosbridge_ip="ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:="

alias test_takeoff="ros2 action send_goal /vhcl0/takeoff auspex_msgs/action/Takeoff '{takeoff_height: 300}'"
alias test_land="ros2 action send_goal /vhcl0/land auspex_msgs/action/Land '{land_speed: 1}'"

alias edit_alias="nano $AUSPEX_HOME/utils/auspex_aliases.sh"
alias reload_alias="source $AUSPEX_HOME/utils/auspex_aliases.sh"

alias edit_exports="nano $AUSPEX_HOME/utils/auspex_exports.sh"
alias reload_exports="source $AUSPEX_HOME/utils/auspex_exports.sh"

alias edusrexp="nano $AUSPEX_HOME/utils/user_exports.sh"

alias source_auspex_exec="source $AUSPEX_HOME/AUSPEX-EXEC/install/setup.bash"
alias source_auspex_plan="source $AUSPEX_HOME/AUSPEX-PLAN/install/setup.bash"
alias source_auspex_sens="source $AUSPEX_HOME/AUSPEX-SENS/install/setup.bash"
alias source_auspex_aero="source $AUSPEX_HOME/AUSPEX-AERO/install/setup.bash"
alias source_auspex_msgs="source $AUSPEX_HOME/AUSPEX-MSGS/install/setup.bash"
alias source_auspex_know="source $AUSPEX_HOME/AUSPEX-KNOW/install/setup.bash"

# source MENTHON workspace
alias source_menthon_ws="source /root/MENTHON-WS/install/setup.bash"

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

alias buinvasa="invasa rebuild_auspex"

alias eb='sudo nano ~/.bashrc'
alias sb='source ~/.bashrc'

alias set_msg_ctx="$AUSPEX_HOME/utils/dev_scripts/set_msg_context.sh"
alias set_disc_server="$AUSPEX_HOME/utils/dev_scripts/set_discovery_server.sh"

alias aero='cd ~/AUSPEX/AUSPEX-AERO/'
alias vasa='cd ~/AUSPEX/AUSPEX-VASA/'
alias sens='cd ~/AUSPEX/AUSPEX-SENS/'
alias msgs='cd ~/AUSPEX/AUSPEX-MSGS/'
alias exec='cd ~/AUSPEX/AUSPEX-EXEC'
alias plan='cd ~/AUSPEX/AUSPEX-PLAN'
alias know='cd ~/AUSPEX/AUSPEX-KNOW'
alias auspex='cd ~/AUSPEX'
