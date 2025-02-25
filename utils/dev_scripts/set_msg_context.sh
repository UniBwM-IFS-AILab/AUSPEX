#!/bin/bash
if [ -z "$1" ]; then
    echo "please use AUSPEX or MENTHON as argument."
    exit 1
fi

if [ "$MSG_CONTEXT" = "AUSPEX" ] && [ "$1" = "MENTHON" ]; then
    echo "switching context from AUSPEX to MENTHON"
    sed -i '/export MSG_CONTEXT=AUSPEX/c\'"export MSG_CONTEXT=MENTHON" "$HOME/auspex_exports.sh"
elif [ "$MSG_CONTEXT" = "MENTHON" ] && [ "$1" = "AUSPEX" ]; then
    echo "switching context from MENTHON to AUSPEX"
    sed -i '/export MSG_CONTEXT=MENTHON/c\'"export MSG_CONTEXT=AUSPEX" "$HOME/auspex_exports.sh"
else
    echo "no change in context or unknown context"
    exit 1
fi

echo "cleaning build, install and log folders..."
SCRIPT_DIR=$(dirname "$(realpath "$0")")
cd $SCRIPT_DIR
cd ..
cd ..
cd AUSPEX-AERO
sudo rm -r ./build
sudo rm -r ./install
sudo rm -r ./log
if [ $AUSPEX_PLATFORM == $VARIANT_TERRA ]; then
    cd ..
    cd AUSPEX-KNOW
    sudo rm -r ./build
    sudo rm -r ./install
    sudo rm -r ./log
fi

echo "please close this shell - open a new one and rebuild_auspex"
