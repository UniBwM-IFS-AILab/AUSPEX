#!/bin/bash
# a script to clean all AUSPEX-x submodules folder for a clean rebuild

AUSPEX_HOME=~/AUSPEX

# clean MSG
echo "cleaning AUSPEX-MSGS"
cd $AUSPEX_HOME/AUSPEX-MSGS
sudo rm -r ./build
sudo rm -r ./install
sudo rm -r ./log

# clean OBC
echo "cleaning AUSPEX-AERO"
cd $AUSPEX_HOME/AUSPEX-AERO/
sudo rm -r ./build
sudo rm -r ./install
sudo rm -r ./log

if [ $AUSPEX_PLATFORM == $VARIANT_TERRA ]; then
    # clean EXE
    echo "cleaning AUSPEX-EXEC"
    cd $AUSPEX_HOME/AUSPEX-EXEC
    sudo rm -r ./build
    sudo rm -r ./install
    sudo rm -r ./log

    # clean PLN
    echo "cleaning AUSPEX-PLAN"
    cd $AUSPEX_HOME/AUSPEX-PLAN
    sudo rm -r ./build
    sudo rm -r ./install
    sudo rm -r ./log

     # clean DPM
    echo "cleaning AUSPEX-SENS"
    cd $AUSPEX_HOME/AUSPEX-SENS
    sudo rm -r ./build
    sudo rm -r ./install
    sudo rm -r ./log

    # clean WKB
    echo "cleaning AUSPEX-KNOW"
    cd $AUSPEX_HOME/AUSPEX-KNOW
    sudo rm -r ./build
    sudo rm -r ./install
    sudo rm -r ./log
fi

cd $AUSPEX_HOME
