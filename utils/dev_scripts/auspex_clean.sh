#!/bin/bash
# a script to clean all AUSPEX-x submodules folder for a clean rebuild

AUSPEX_HOME=/root/AUSPEX

# clean MSG
echo "cleaning AUSPEX-MSGS"
cd $AUSPEX_HOME/AUSPEX-MSGS
rm -r ./build
rm -r ./install
rm -r ./log

# clean AERO
echo "cleaning AUSPEX-AERO"
cd $AUSPEX_HOME/AUSPEX-AERO/
rm -r ./build
rm -r ./install
rm -r ./log

echo "cleaning AUSPEX-KNOW"
cd $AUSPEX_HOME/AUSPEX-KNOW
rm -r ./build
rm -r ./install
rm -r ./log

# clean EXE
echo "cleaning AUSPEX-EXEC"
cd $AUSPEX_HOME/AUSPEX-EXEC
rm -r ./build
rm -r ./install
rm -r ./log

# clean PLAN
echo "cleaning AUSPEX-PLAN"
cd $AUSPEX_HOME/AUSPEX-PLAN
rm -r ./build
rm -r ./install
rm -r ./log

    # clean SENS
echo "cleaning AUSPEX-SENS"
cd $AUSPEX_HOME/AUSPEX-SENS
rm -r ./build
rm -r ./install
rm -r ./log

cd $AUSPEX_HOME
