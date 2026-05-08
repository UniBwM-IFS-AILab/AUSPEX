#!/bin/bash
# a script to pull all AUSPEX-x submodules
AUSPEX_HOME=~/AUSPEX

cd $AUSPEX_HOME
echo "pulling AUSPEX"
git pull

# pull MSGS
echo "pulling AUSPEX-MSGS"
cd $AUSPEX_HOME/AUSPEX-MSGS
git pull

# pull AERO
echo "pulling AUSPEX-AERO"
cd $AUSPEX_HOME/AUSPEX-AERO
git pull

echo "pulling AUSPEX-VASA"
cd $AUSPEX_HOME/AUSPEX-VASA
git pull

echo "pulling AUSPEX-KNOW"
cd $AUSPEX_HOME/AUSPEX-KNOW
git pull

# pull EXEC
echo "pulling AUSPEX-EXEC"
cd $AUSPEX_HOME/AUSPEX-EXEC
git pull

# pull PLAN
echo "pulling AUSPEX-PLAN"
cd $AUSPEX_HOME/AUSPEX-PLAN
git pull

# pull SENS
echo "pulling AUSPEX-SENS"
cd $AUSPEX_HOME/AUSPEX-SENS
git pull

cd $AUSPEX_HOME
