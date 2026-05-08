#!/bin/bash
# a script to build all AUSPEX-x submodules
AUSPEX_HOME=/root/AUSPEX/
AUSPEX_UTILS=/root/AUSPEX/utils/dev_scripts
cd $AUSPEX_UTILS
./auspex_clean.sh && ./auspex_build.sh
cd $AUSPEX_HOME
