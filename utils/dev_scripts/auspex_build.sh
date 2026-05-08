#!/bin/bash
# a script to build all AUSPEX-x submodules
AUSPEX_HOME=/root/AUSPEX

# build MSGS
echo "building AUSPEX-MSGS"
cd $AUSPEX_HOME/AUSPEX-MSGS
colcon build --allow-overriding auspex_msgs
source install/setup.bash

# build AERO
echo "building AUSPEX-AERO"
cd $AUSPEX_HOME/AUSPEX-AERO/
colcon build

echo "building AUSPEX-KNOW"
cd $AUSPEX_HOME/AUSPEX-KNOW
colcon build

# build EXE
echo "building AUSPEX-EXEC"
cd $AUSPEX_HOME/AUSPEX-EXEC
colcon build

# build SENS
echo "building AUSPEX-SENS"
cd $AUSPEX_HOME/AUSPEX-SENS
colcon build

# build PLAN
echo "building AUSPEX-PLAN"
cd $AUSPEX_HOME/AUSPEX-PLAN
colcon build

cd $AUSPEX_HOME
