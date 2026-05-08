# start complete simulation
export AUSPEX_HOST_DIR="/home/$USER/AUSPEX"

alias auspex_run_sim_ue="$AUSPEX_HOST_DIR/utils/startup_scripts/auspex_run_sim.sh ue"
alias auspex_run_sim_is="$AUSPEX_HOST_DIR/utils/startup_scripts/auspex_run_sim.sh is"
alias auspex_run_gcs="$AUSPEX_HOST_DIR/utils/startup_scripts/auspex_run_sim.sh gcs"

alias auspex_kill="iv 'kill_px4 && pkill -f copernicus_serv && pkill -f valkey-server && pkill -f knowledge_main && pkill -f auspex_obc && pkill -f upf4ros2_main && pkill -f executor_main_n && pkill -f planning_main_n'"
alias auspex_install="cd $AUSPEX_HOST_DIR/ && ./install.sh TERRA"
alias auspex_setup="cd $AUSPEX_HOST_DIR/ && ./setup.py"
alias auspex_pull="$AUSPEX_HOST_DIR/utils/dev_scripts/auspex_pull.sh"

alias edit_properties="nano $AUSPEX_HOST_DIR/params/platform_properties/platform_properties.json"

# Directory Aliases
alias aero="cd $AUSPEX_HOST_DIR/AUSPEX-AERO/"
alias vasa="cd $AUSPEX_HOST_DIR/AUSPEX-VASA/"
alias sens="cd $AUSPEX_HOST_DIR/AUSPEX-SENS/"
alias msgs="cd $AUSPEX_HOST_DIR/AUSPEX-MSGS/"
alias exec="cd $AUSPEX_HOST_DIR/AUSPEX-EXEC"
alias plan="cd $AUSPEX_HOST_DIR/AUSPEX-PLAN"
alias know="cd $AUSPEX_HOST_DIR/AUSPEX-KNOW"
alias auspex="cd $AUSPEX_HOST_DIR"
alias params="cd $AUSPEX_HOST_DIR/params"

alias auspex_vpn_run="$AUSPEX_HOST_DIR/utils/ovpn/connect-vpn.sh"
alias auspex_vpn_stop="sudo pkill -SIGINT openvpn"
alias ssh_connect="$AUSPEX_HOST_DIR/utils/startup_scripts/auspex_ssh_connect.sh"

source "$AUSPEX_HOST_DIR/utils/user_exports.sh"
