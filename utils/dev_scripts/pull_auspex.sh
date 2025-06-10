#!/bin/bash
# a script to pull all AUSPEX-x submodules
AUSPEX_HOME=~/AUSPEX

# pull MSG
echo "building AUSPEX-MSGS"
cd $AUSPEX_HOME/AUSPEX-MSGS
git pull

# pull OBC
echo "building AUSPEX-AERO"
cd $AUSPEX_HOME/AUSPEX-AERO
git pull

echo "building AUSPEX-VASA"
cd $AUSPEX_HOME/AUSPEX-VASA
git pull

if [ $AUSPEX_PLATFORM == "TERRA" ]; then
    # pull EXE
    echo "building AUSPEX-EXEC"
    cd $AUSPEX_HOME/AUSPEX-EXEC
    git pull

    # pull PLN
    echo "building AUSPEX-PLAN"
    cd $AUSPEX_HOME/AUSPEX-PLAN
    git pull

    # pull DPM
    echo "building AUSPEX-SENS"
    cd $AUSPEX_HOME/AUSPEX-SENS
    git pull

    # pull WKB
    echo "building AUSPEX-KNOW"
    cd $AUSPEX_HOME/AUSPEX-KNOW
    git pull
fi

cd $AUSPEX_HOME
