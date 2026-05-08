# contains general environment variables for AUSPEX

# Setup to omit warnings about parallelism in tokenizers
export TOKENIZERS_PARALLELISM=false
export XDG_RUNTIME_DIR=/tmp/runtime-companion
export RUNLEVEL=3

# Python warnings
export PYTHONWARNINGS="ignore:setup.py install is deprecated,ignore:easy_install command is deprecated"

# source user-specific variables
source $AUSPEX_HOME/utils/user_exports.sh

export PX4_SIM_MODEL=none_iris

export AUSPEX_PARAMS_PATH="$AUSPEX_HOME/params"

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

msgs_source
aero_source
know_source
exec_source
plan_source
sens_source

if [ "$OBC_TYPE" == "DESKTOP" ]; then
    # For FastDDS <-> GStreamer Conflict on DESKTOP
    export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libunwind.so.8
fi
