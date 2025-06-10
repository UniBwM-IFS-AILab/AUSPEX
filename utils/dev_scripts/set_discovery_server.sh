#!/bin/bash
if [ -z "$1" ]; then
    echo "please use [AUSPEX | MENTHON | LOCAL] as argument."
    exit 1
fi

echo "switching discovery server to $1"
sed -i "s|^export DISCOVERY_SERVER=.*|export DISCOVERY_SERVER=$1|" "$HOME/AUSPEX/utils/user_exports.sh"

echo "please close this shell or source ~/.bashrc"
