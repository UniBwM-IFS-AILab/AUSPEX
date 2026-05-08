# path to AUSPEX base folder
AUSPEX_HOME="/root/AUSPEX"

# start/stop PX4 autopilot
alias px4_kill="pkill -x px4 || true"
alias px4_run="/root/scripts/sitl_multiple_run.sh"

alias listen="ros2 run demo_nodes_cpp listener"
alias talk="ros2 run demo_nodes_cpp talker"

# start height server
alias height_server_run="ros2 run auspex_knowledge height_server"

# start/stop valkey
alias valkey_run="$AUSPEX_HOME/utils/valkey/valkey_run.sh"
alias valkey_stop="pkill valkey-server"

# start AUSPEX_KNOW
alias know_run="sleep 3; ros2 run auspex_knowledge knowledge_main"

# start AUSPEX_AERO
alias aero_run="ros2 launch auspex_aero auspex_aero.launch.py"

# start AUSPEX_CTRL
alias ctrl_run="sleep 3; ros2 launch auspex_executor controller_main.launch.py"

# start AUSPEX_EXEC on a platform
alias exec_run="sleep 3; ros2 launch auspex_executor decentral_executor_main.launch.py"

# start AUSPEX_PLAN
alias plan_run="ros2 launch auspex_planning planning_main.launch.py"

# start AUSPEX_SENS
alias sens_run="ros2 run auspex_perception sens_main"

# start command publisher
alias cmd_run="ros2 run auspex_planning planning_command_publisher"
alias cmd_plan="ros2 topic pub --once planner_command auspex_msgs/msg/UserCommand \"{user_command: 7, team_id: 'drone_team'}\""
alias cmd_accept="ros2 topic pub --once planner_command auspex_msgs/msg/UserCommand \"{user_command: 5, team_id: 'drone_team'}\""

# start rosbridge server
alias bridge_run="ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
alias bridge_run_ip="ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:="

alias exec_source="source $AUSPEX_HOME/AUSPEX-EXEC/install/setup.bash"
alias plan_source="source $AUSPEX_HOME/AUSPEX-PLAN/install/setup.bash"
alias sens_source="source $AUSPEX_HOME/AUSPEX-SENS/install/setup.bash"
alias aero_source="source $AUSPEX_HOME/AUSPEX-AERO/install/setup.bash"
alias msgs_source="source $AUSPEX_HOME/AUSPEX-MSGS/install/setup.bash"
alias know_source="source $AUSPEX_HOME/AUSPEX-KNOW/install/setup.bash"

alias exec_build="cd $AUSPEX_HOME/AUSPEX-EXEC && colcon build"
alias plan_build="cd $AUSPEX_HOME/AUSPEX-PLAN && colcon build"
alias sens_build="cd $AUSPEX_HOME/AUSPEX-SENS && colcon build"
alias msgs_build="cd $AUSPEX_HOME/AUSPEX-MSGS && colcon build"
alias aero_build="cd $AUSPEX_HOME/AUSPEX-AERO && colcon build"
alias know_build="cd $AUSPEX_HOME/AUSPEX-KNOW && colcon build"

alias auspex_rebuild="$AUSPEX_HOME/utils/dev_scripts/auspex_rebuild.sh"
alias auspex_build="$AUSPEX_HOME/utils/dev_scripts/auspex_build.sh"
alias auspex_clean="$AUSPEX_HOME/utils/dev_scripts/auspex_clean.sh"

alias eb='nano ~/.bashrc'
alias sb='source ~/.bashrc'

alias aero='cd ~/AUSPEX/AUSPEX-AERO/'
alias vasa='cd ~/AUSPEX/AUSPEX-VASA/'
alias sens='cd ~/AUSPEX/AUSPEX-SENS/'
alias msgs='cd ~/AUSPEX/AUSPEX-MSGS/'
alias exec='cd ~/AUSPEX/AUSPEX-EXEC'
alias plan='cd ~/AUSPEX/AUSPEX-PLAN'
alias know='cd ~/AUSPEX/AUSPEX-KNOW'
alias auspex='cd ~/AUSPEX'
alias params="cd ~/AUSPEX/params"
