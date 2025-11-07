#!/bin/bash
# a script to pull all AUSPEX-x submodules
AUSPEX_HOME=~/AUSPEX

cd $AUSPEX_HOME
echo "pulling AUSPEX"
git pull

# pull MSG
echo "pulling AUSPEX-MSGS"
cd $AUSPEX_HOME/AUSPEX-MSGS
git pull

# pull OBC
echo "pulling AUSPEX-AERO"
cd $AUSPEX_HOME/AUSPEX-AERO
git pull

echo "pulling AUSPEX-VASA"
cd $AUSPEX_HOME/AUSPEX-VASA
git pull

if [ $AUSPEX_PLATFORM == "TERRA" ]; then
    # pull EXE
    echo "pulling AUSPEX-EXEC"
    cd $AUSPEX_HOME/AUSPEX-EXEC
    git pull

    # pull PLN
    echo "pulling AUSPEX-PLAN"
    cd $AUSPEX_HOME/AUSPEX-PLAN
    git pull

    # pull DPM
    echo "pulling AUSPEX-SENS"
    cd $AUSPEX_HOME/AUSPEX-SENS
    git pull

    # pull WKB
    echo "pulling AUSPEX-KNOW"
    cd $AUSPEX_HOME/AUSPEX-KNOW
    git pull
fi

cd $AUSPEX_HOME
