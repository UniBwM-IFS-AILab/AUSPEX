# start complete simulation
AUSPEX_HOST_DIR="/home/$USER/AUSPEX"
alias win_start_sim="$AUSPEX_HOST_DIR/utils/startup_scripts/win_start_sim.sh"
alias win_start_gcs="$AUSPEX_HOST_DIR/utils/startup_scripts/win_start_gcs.sh"
alias kill_sim="iv 'kill_px4 && pkill -f MicroXRCEAgent && pkill -f copernicus_serv && pkill -f valkey-server && pkill -f knowledge_main && pkill -f auspex_obc && pkill -f upf4ros2_main && pkill -f image_processin && pkill -f executor_main_n && pkill -f planning_main_n'"
alias install_auspex="cd $AUSPEX_HOST_DIR/ && ./install.sh TERRA"
alias setup_auspex="cd $AUSPEX_HOST_DIR/ && ./setup.sh"

# Directory Aliases
alias aero="cd $AUSPEX_HOST_DIR/AUSPEX-AERO/"
alias vasa="cd $AUSPEX_HOST_DIR/AUSPEX-VASA/"
alias sens="cd $AUSPEX_HOST_DIR/AUSPEX-SENS/"
alias msgs="cd $AUSPEX_HOST_DIR/AUSPEX-MSGS/"
alias exec="cd $AUSPEX_HOST_DIR/AUSPEX-EXEC"
alias plan="cd $AUSPEX_HOST_DIR/AUSPEX-PLAN"
alias know="cd $AUSPEX_HOST_DIR/AUSPEX-KNOW"
alias auspex="cd ~/AUSPEX"