# for tokenizer
export TOKENIZERS_PARALLELISM=false

# for gui applications
export XDG_RUNTIME_DIR=/tmp/runtime-companion
export RUNLEVEL=3

# manually update px4_sim_host_addr when wsl started and ipconfig on windows with wsl ip address
export PX4_SIM_MODEL=none_iris
export PX4_SIM_HOST_ADDR=172.17.208.1
export WSL_HOST_IP=172.17.208.1 # $(cat /etc/resolv.conf | grep nameserver | awk "{print $2}")
export PATH="$PATH:/home/companion/.local/bin"
export PATH="$PATH:/home/companion/.local/lib/python3.10/site-packages"

export AUSPEX_PARAMS_PATH="$HOME/auspex_params"

# ignore setuptools warning for ROS2
export PYTHONWARNINGS="ignore:setup.py install is deprecated,ignore:easy_install command is deprecated"

# DDS stuff
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=~/AUSPEX/utils/dds_scripts/fastrtps_dds_tailscale/fastrtps_dds_setup.xml

if [ "$MSG_CONTEXT" == "MENTHON" ]; then
    export ROS_DOMAIN_ID=90
else
    export ROS_DOMAIN_ID=0
fi

source_all
