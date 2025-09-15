# contains general environment variables for AUSPEX

# source user-specific variables
source $AUSPEX_HOME/utils/user_exports.sh

export TOKENIZERS_PARALLELISM=false

export XDG_RUNTIME_DIR=/tmp/runtime-companion
export RUNLEVEL=3

export PX4_SIM_MODEL=none_iris

export AUSPEX_PARAMS_PATH="$HOME/auspex_params"

export PYTHONWARNINGS="ignore:setup.py install is deprecated,ignore:easy_install command is deprecated"

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

if [ "$AUSPEX_PLATFORM" == "TERRA" ]; then
    source_auspex_plan
    source_auspex_exec
    source_auspex_sens
    source_auspex_aero
    source_auspex_know
    source_auspex_msgs
else
    source_auspex_msgs
    source_auspex_aero
fi
