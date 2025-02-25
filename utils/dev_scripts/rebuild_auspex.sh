#!/bin/bash
# a script to build all AUSPEX-x submodules
AUSPEX_HOME=~/AUSPEX

# build MSG
echo "building AUSPEX-MSGS"
cd $AUSPEX_HOME/AUSPEX-MSGS
colcon build --allow-overriding auspex_msgs
source install/setup.bash

# build OBC
echo "building AUSPEX-AERO"
cd $AUSPEX_HOME/AUSPEX-AERO/
colcon build

if [ $AUSPEX_PLATFORM == $VARIANT_TERRA ]; then
    # build EXE
    echo "building AUSPEX-EXEC"
    cd $AUSPEX_HOME/AUSPEX-EXEC
    colcon build

    # build DPM
    echo "building AUSPEX-SENS"
    cd $AUSPEX_HOME/AUSPEX-SENS
    colcon build

    # build PLN
    echo "building AUSPEX-PLAN"
    cd $AUSPEX_HOME/AUSPEX-PLAN
    colcon build

    # build WKB
    echo "building AUSPEX-KNOW"
    cd $AUSPEX_HOME/AUSPEX-KNOW
    colcon build
fi

cd $AUSPEX_HOME
