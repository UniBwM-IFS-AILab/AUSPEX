#!/bin/bash
# a script to build all AUSPEX-x submodules
AUSPEX_HOME=/root/AUSPEX/
AUSPEX_UTILS=/root/AUSPEX/utils/dev_scripts
cd $AUSPEX_UTILS
./clean_auspex.sh && ./build_auspex.sh
cd $AUSPEX_HOME
