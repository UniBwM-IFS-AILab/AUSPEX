#!/bin/bash
# a script to set up the environment for the AUSPEX Framework
# pull the submodules
# install dependencies
# modify configs of external submodules
# setup enviroment variables and aliases
# build all submodules

UTILS_DIR=$(dirname "$(realpath "$0")")/utils
TOP_LEVEL_DIR=$(dirname "$UTILS_DIR")

VARIANT_TERRA="TERRA" # desktop variant
VARIANT_AVIS="AVIS" # drone variant

FC_VARIANT_PX4="PX4" 
FC_VARIANT_ANAFI="ANAFI"

FC_VERSION=$FC_VARIANT_PX4 # is appended with _SIMULATION if AUSPEX variant is selected

ROS_VERSION="humble"

# function to add lines to ~/.bashrc if they are not already present
add_to_file_if_not_present() {
    local line="$1"
    local file="$2"

    # trim whitespace from the start and end of the line for comparison purposes
    local trimmed_line
    trimmed_line=$(echo "$line" | xargs)

    # ensure the file ends with a newline before processing
    if [ -f "$file" ] && [ "$(tail -c 1 "$file")" != "" ]; then
        echo "" >> "$file"
    fi

    # Check if the line already exists in the file
    if ! grep -Fxq "$trimmed_line" "$file"; then
        echo "$trimmed_line" >> "$file"
    fi
}

install_pip_package() {
    package=$1
    echo "installing(pip): $package ..."
    if ! pip3 install -q "$package"; then
        echo "error: Failed to install $package" >&2
        exit 1
    fi
}

apt_update() {
    echo "running sudo apt update ..."
    # run the update and redirect output to the log file
    if ! sudo apt -y -qq update > /dev/null 2>&1; then
        echo "error: failed to update package lists." >&2
        exit 1
    fi
}

colcon_build(){
    echo "running colcon build ..."
    if ! colcon build > /dev/null 2>&1; then
        echo "error: colcon_build failed. Check the build configuration or dependencies." >&2
        exit 1
    fi
}

apt_upgrade() {
    echo "running sudo apt upgrade ..."
    # Run the update and redirect output to the log file
    if ! sudo apt -y -qq upgrade > /dev/null 2>&1; then
        echo "error: failed to upgrade package lists. " >&2
        exit 1
    fi
}

install_apt_package() {
    package=$1
    echo "installing(apt): $package ..."
    if ! sudo apt -y -qq install "$package" > /dev/null 2>&1; then
        echo "error: failed to install $package" >&2
        exit 1
    fi
}

# check if variant is provided as command line argument
if [ -z "$1" ]; then
    echo "no argument provided. Please start script with either $VARIANT_TERRA or $VARIANT_AVIS as argument."
    exit 1
fi

AUSPEX_PLATFORM=$1

# check if variant is supported (TERRA or AVIS)
if [ $AUSPEX_PLATFORM != $VARIANT_TERRA ] && [ $AUSPEX_PLATFORM != $VARIANT_AVIS ]; then
    echo "The string "$AUSPEX_PLATFORM" is not supported. Please enter TERRA or AVIS."
    exit 1
fi

# make simulation start scripts executable
sudo chmod +x $UTILS_DIR/startup_scripts/start_simulation.sh

# copy alias files to home
cp $UTILS_DIR/auspex_aliases.sh ~/
cp $UTILS_DIR/auspex_exports.sh ~/

# add python path
if ! grep -q "export PYTHONPATH=" "$HOME/.bashrc"; then
    add_to_file_if_not_present 'export PYTHONPATH="$PYTHONPATH:/home/companion/.local/lib/python3.10/site-packages"' ~/.bashrc
fi

# adding sourcing of AUSPEX files to .bashrc
add_to_file_if_not_present "source ~/auspex_aliases.sh" ~/.bashrc
add_to_file_if_not_present "source ~/auspex_exports.sh" ~/.bashrc
add_to_file_if_not_present "export AUSPEX_PLATFORM=$AUSPEX_PLATFORM" ~/auspex_exports.sh
add_to_file_if_not_present "export VARIANT_TERRA=$VARIANT_TERRA" ~/auspex_exports.sh
add_to_file_if_not_present "export VARIANT_AVIS=$VARIANT_AVIS" ~/auspex_exports.sh

if [ $AUSPEX_PLATFORM == $VARIANT_TERRA ]; then
    FC_VERSION="${FC_VERSION}_SIMULATION"
    add_to_file_if_not_present "export OBC_MODE=$FC_VERSION" ~/auspex_exports.sh
    add_to_file_if_not_present "source_auspex_plan" ~/auspex_exports.sh
    add_to_file_if_not_present "source_auspex_exec" ~/auspex_exports.sh
    add_to_file_if_not_present "source_auspex_sens" ~/auspex_exports.sh
    add_to_file_if_not_present "source_auspex_aero" ~/auspex_exports.sh
    add_to_file_if_not_present "source_auspex_know" ~/auspex_exports.sh
    add_to_file_if_not_present "source_auspex_msgs" ~/auspex_exports.sh
elif [ $AUSPEX_PLATFORM == $VARIANT_AVIS ]; then
    add_to_file_if_not_present "export OBC_MODE=$FC_VERSION" ~/auspex_exports.sh
    add_to_file_if_not_present "source_auspex_msgs" ~/auspex_exports.sh
    add_to_file_if_not_present "source_auspex_aero" ~/auspex_exports.sh
fi

# checking for msg context
if [ -z "$MSG_CONTEXT" ]; then
    echo "MSG_CONTEXT not configured. Setting to AUSPEX..."
    MSG_CONTEXT="AUSPEX"
    add_to_file_if_not_present "export MSG_CONTEXT="AUSPEX"" ~/auspex_exports.sh
fi

# updating the submodules respective to the platform in use
if [ $AUSPEX_PLATFORM == $VARIANT_TERRA ]; then
    echo "cloning repository for "down to earth""
    export OBC_MODE=$FC_VERSION
    OBC_MODE=$FC_VERSION
    cd $TOP_LEVEL_DIR && git submodule update --init --recursive AUSPEX-PLAN AUSPEX-EXEC AUSPEX-MSGS AUSPEX-KNOW AUSPEX-AERO AirSim PX4-Autopilot Micro-XRCE-DDS-Agent

elif [ $AUSPEX_PLATFORM == $VARIANT_AVIS ]; then
    echo "cloning repository for "in the sky""
    export OBC_MODE=$FC_VERSION
    cd $TOP_LEVEL_DIR && git submodule update --init --recursive AUSPEX-MSGS AUSPEX-AERO Micro-XRCE-DDS-Agent
    OBC_MODE=$FC_VERSION
    ROS_VERSION="jazzy"
fi

# Add source ros to bash
add_to_file_if_not_present "source /opt/ros/$ROS_VERSION/setup.bash" ~/.bashrc

# install ROS2 
install_apt_package software-properties-common
if ! grep -qq "^deb .*universe" /etc/apt/sources.list /etc/apt/sources.list.d/*; then
    echo "adding the 'universe' repository..."
    sudo add-apt-repository universe -y -qq
fi

apt_update && install_apt_package curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
apt_update
apt_upgrade
install_apt_package ros-$ROS_VERSION-desktop
install_apt_package ros-$ROS_VERSION-geographic-msgs
install_apt_package ros-$ROS_VERSION-tf-transformations
install_apt_package ros-$ROS_VERSION-rosbridge-suite

# install tailscale
if [ $ROS_VERSION == "jazzy" ]; then
    curl -fsSL https://pkgs.tailscale.com/stable/ubuntu/noble.noarmor.gpg | sudo tee /usr/share/keyrings/tailscale-archive-keyring.gpg >/dev/null
    curl -fsSL https://pkgs.tailscale.com/stable/ubuntu/noble.tailscale-keyring.list | sudo tee /etc/apt/sources.list.d/tailscale.list >/dev/null
else 
    curl -fsSL https://pkgs.tailscale.com/stable/ubuntu/jammy.noarmor.gpg | sudo tee /usr/share/keyrings/tailscale-archive-keyring.gpg >/dev/null
    curl -fsSL https://pkgs.tailscale.com/stable/ubuntu/jammy.tailscale-keyring.list | sudo tee /etc/apt/sources.list.d/tailscale.list >/dev/null
fi

apt_update
install_apt_package tailscale

# init dds config for fast rtps
source ~/AUSPEX/utils/dds_scripts/fastrtps_dds_tailscale/update_fastdds_xml.sh LOCAL

# json
install_apt_package nlohmann-json3-dev

# install packages needed for building
install_apt_package python3-colcon-common-extensions
install_apt_package python3-pip
install_apt_package libunwind-dev libstdc++-12-dev

pip3 install --upgrade pip

# pip3 install symforce==0.9.0
if [ $AUSPEX_PLATFORM == $VARIANT_TERRA ]; then
    # a torch version is required
    install_pip_package msgpack-rpc-python
    install_pip_package "airsim==1.8.1"
    install_pip_package pandas
    install_pip_package alns
    install_pip_package scikit-learn
    install_pip_package google-generativeai
    install_pip_package anthropic
    install_pip_package openai
    install_pip_package "transformers==4.46.3"
    install_pip_package unified-planning[engines]
fi
install_pip_package ultralytics
install_pip_package webcolors
install_pip_package "scipy==1.15.1"
install_pip_package "numpy==1.24.0" > /dev/null 2>&1
install_pip_package pyparsing
install_pip_package sympy

install_pip_package requests
install_pip_package oauthlib
install_pip_package requests-oauthlib
install_pip_package rasterio

install_pip_package kconfiglib
install_pip_package future 
install_pip_package pyros-genmsg
install_pip_package "transforms3d==0.4.2"
install_pip_package geopy
install_pip_package jsonschema
install_pip_package valkey
install_pip_package shapely
install_pip_package "tornado==6.4.2"

# source ROS
. /opt/ros/$ROS_VERSION/setup.bash

# build AUSPEX-MSGS 
cd $TOP_LEVEL_DIR/AUSPEX-MSGS 
git checkout main > /dev/null 2>&1
git submodule update --init --recursive

cd src/UP4ROS2
git sparse-checkout init --cone
git sparse-checkout set up_msgs
cd $TOP_LEVEL_DIR/AUSPEX-MSGS
colcon_build
source install/setup.bash

# build packages for desktop
if [ $AUSPEX_PLATFORM == $VARIANT_TERRA ]; then
    echo "setting up desktop environment..."

    # build PX4
    echo "installing(git): PX4 ..."
    cd $TOP_LEVEL_DIR/PX4-Autopilot
    git checkout -f v1.14.2 > /dev/null 2>&1
    git submodule update --init --recursive
    cp $UTILS_DIR/copy_files/dds_topics.yaml $TOP_LEVEL_DIR/PX4-Autopilot/src/modules/uxrce_dds_client/

    make px4_sitl_default > /dev/null 

    mkdir $TOP_LEVEL_DIR/PX4-Autopilot/scripts 2>/dev/null
    cp $UTILS_DIR/mod_airsim_files/sitl_multiple_run.sh $TOP_LEVEL_DIR/PX4-Autopilot/Tools/simulation/

    # build AirSim
    echo "installing(git): AirSim ..."
    cd $TOP_LEVEL_DIR/AirSim

    # overwrite some AirSim files (See: https://github.com/microsoft/AirSim/issues/4892)
    cp -f $UTILS_DIR/mod_airsim_files/setup.sh $TOP_LEVEL_DIR/AirSim/
    cp -f $UTILS_DIR/mod_airsim_files/build.sh $TOP_LEVEL_DIR/AirSim/
    cp -f $UTILS_DIR/mod_airsim_files/cmake/cmake-modules/CommonSetup.cmake $TOP_LEVEL_DIR/AirSim/cmake/cmake-modules/
    cp -f $UTILS_DIR/mod_airsim_files/ros2/src/airsim_ros_pkgs/CMakeLists.txt $TOP_LEVEL_DIR/AirSim/ros2/src/airsim_ros_pkgs/
    cp -f $UTILS_DIR/mod_airsim_files/Unity/build.sh $TOP_LEVEL_DIR/AirSim/Unity/

    # check if the copy was successful
    if [ $? -eq 0 ]; then
        echo "files copied successfully."
    else
        echo "error copying the files."
    fi

    $TOP_LEVEL_DIR/AirSim/setup.sh > /dev/null 2>&1
    $TOP_LEVEL_DIR/AirSim/build.sh > /dev/null 2>&1

    # build AUSPEX-SENS
    echo "installing(git): SENS ..."
    cd $TOP_LEVEL_DIR/AUSPEX-SENS
    git checkout main > /dev/null 2>&1
    git submodule update --init --recursive
    colcon_build
    source install/setup.bash

    # build AUSPEX-PLAN
    echo "installing(git): PLAN ..."
    cd $TOP_LEVEL_DIR/AUSPEX-PLAN
    git checkout main > /dev/null 2>&1
    git submodule update --init --recursive
    rm -rf src/UP4ROS2/up_msgs
    colcon_build
    source install/setup.bash

    # build AUSPEX-EXEC
    echo "installing(git): EXEC ..."
    cd $TOP_LEVEL_DIR/AUSPEX-EXEC
    git checkout main > /dev/null 2>&1
    colcon_build
    source install/setup.bash

    # build AUSPEX-KNOW
    echo "installing(git): KNOW ..."
    cd $TOP_LEVEL_DIR/AUSPEX-KNOW
    git checkout main > /dev/null 2>&1

    git submodule update --init --recursive valkey/valkey valkey/valkeyJSON
    # build valkey
    cd valkey
    IGNORE_FILE="COLCON_IGNORE"
    if [ -f "$IGNORE_FILE" ]; then
        echo "COLCON_IGNORE already exists"
    else
        touch "$IGNORE_FILE"
    fi

    cd valkey
    make > /dev/null 2>&1
    # build valkey json module
    cd ..
    cd valkeyJSON
    OVERRIDE_LINE="OVERRIDE_FIND_PACKAGE"
    BRACKET_LINE=""
    sed -i "s|$OVERRIDE_LINE|$BRACKET_LINE|" ./tst/CMakeLists.txt
    mkdir build > /dev/null 2>&1
    cd build
    cmake .. > /dev/null 2>&1
    make > /dev/null 2>&1

    cd $TOP_LEVEL_DIR/AUSPEX-KNOW
    colcon_build
    source install/setup.bash
else
    # nothing special to build in case of drone installation
    echo "continuing with general setup..."
fi

# build CTRL
echo "installing(git): AUSPEX-AERO ..."

cd $TOP_LEVEL_DIR/AUSPEX-AERO
git checkout main > /dev/null 2>&1
colcon_build
source $TOP_LEVEL_DIR/AUSPEX-AERO/install/setup.bash

# build uXRCE Agent
echo "installing(git): Micro-XRCE-DDS-Agent ..."
cd $TOP_LEVEL_DIR/Micro-XRCE-DDS-Agent 
git checkout -f ros2 > /dev/null 2>&1
cp $UTILS_DIR/copy_files/CMakeLists.txt $TOP_LEVEL_DIR/Micro-XRCE-DDS-Agent/
mkdir build > /dev/null 2>&1
cd build > /dev/null 2>&1
cmake .. > /dev/null 2>&1

make > /dev/null 2>&1
sudo make install > /dev/null 2>&1
sudo ldconfig /usr/local/lib/ > /dev/null 2>&1

# Copy platform properties file
if ! [ -d "$HOME/auspex_params" ]; then
    mkdir $HOME/auspex_params >/dev/null
    cp -r $UTILS_DIR/auspex_params $HOME
fi

if [ $AUSPEX_PLATFORM == $VARIANT_TERRA ]; then
    if ! [[ -f "/etc/wsl.conf" ]]; then
        #  overwrite wsl.conf with custom conf
        echo "cp $UTILS_DIR/copy_files/wsl.conf ~/"
        cp $UTILS_DIR/copy_files/wsl.conf $HOME/

        echo "please enter your new hostname:"
        read name

        add_to_file_if_not_present "hostname=${name}" $HOME/wsl.conf
        add_to_file_if_not_present "[user]" $HOME/wsl.conf
        add_to_file_if_not_present "default=$USER" $HOME/wsl.conf

        sudo mv ~/wsl.conf /etc/wsl.conf
        echo "copied wsl.conf to /etc/wsl.conf"
    else
        echo "wsl.conf does already exists."
    fi
fi

echo ""
echo "please restart WSL (wsl --shutdown in powershell) or OBC and enter "sudo tailscale up" to login to tailscale with current hostname"

echo ""
echo "---------------------------"
echo "finished installing AUSPEX!"
echo "---------------------------"
echo ""
echo " -------------:----------::::::::::::::::::::::::::::::::::::::::::::::::::::::::"
echo " --:**#######.@.@%*++===+:::::::::::::::::::::::::::::::=*#####%@.=@.@%%%##%@%:::"
echo " ---:::--:--::.:::::::::::::::::::::#@@@@@@@+:::::::::::...::::::....::::::::::::"
echo " ---------::@@@@@:::::::::::+@@@@@#.::::::::::.@@@@@#:::::::::::@@@@@+:::::::::::"
echo " ----------@@:::@@::::::-@@@.:::::::::::::::::::::::.@@@*::::::-@...@@:::::::::::"
echo " ----------:@@@@@:::::@@@:::::::::::::::::::::::::::::::+@@..:::%@@@@.:::::::::::"
echo " -----------:::::::@@::::::::::::::::@@@@@@@%::::::::::::::..@%.:::::::::::::::::"
echo " -----------::::::::*@@::::::::::::@@:::::::@@%:::::::::::+@@::::::::::::::::::::"
echo " ------------:::::::::=@@:::::::::@@::.@@@@::.@=::::::::#@@.:::%@-:::::::::::::::"
echo " ---::----:::::::::::::::@@:::::::@@::@@+%@.:.@%::::::#@@.:::::::..::::::::::::::"
echo " ---:::::::::::::::::::::::@@:::::*@*:::*::::@@.::::%@@.:::::::::::::::::::::::::"
echo " :@@@@@@@@@.@.@@@@@@@@@::::::@@::::=@@=::::@@@::::%@@.:::::@@@@@@@@@.@.@@@@@@@@@:"
echo " ::::--::::::.:::::::::::::::::@@.::::%@@@@:::::%@@..:::::::::::::::....:::::::::"
echo " -::::::::@@@@@:::::::::::::::::....::::::::::..:...:::::::::::::::@@@@@:::::::::"
echo " -::::::::::::::::::::::::::::::::.:::::...::::::..:::::::::::::::....:::::::::::"
echo " ::::::::::::@@@@@@@@@@@@@%%#+=@@@@+@@@.::@@@@+==+#%@@%%@@@@@@@@@@@@.:::::.::::::"
echo " :::::#@::::::@@::::::::::@@@@@@:#@@+:::::::=@@@:@@@@@@....::::::@@.:::::@@::::::"
echo " :::::%@:::::::@@@::@@@@@%.::.%@@*:::::::::::..-@@@....%@@@@%.:%@@.::::::@@::::::"
echo " :::::#@::::::::::@@.::::::*@@@:::::@@@@@@@@@*....%@@@..::::.@@#:::::::::@@::::::"
echo " :::::.@::::::::::::@@@::@@@...%@@.:...:.:::...+@@.::@@@...@@%:::::::::::@@::::::"
echo " ::::::@@:::::::::::::-@@@:..@@@::::.@@@@@@@@.:::@@@..::@@#.::::::::::::.@+::::::"
echo " ::::::=@.:::::::::::::::::@@@:.::::....:.....:.::::@@.:::::::::::::::::@@:::::::"
echo " ::::::::::::::::::::::::%@*:::::::::@@:::@@...:::::::%%:..:::::::::::::...::::::"
echo " :::::::::::::*.::::::.....:::.@@-.:.@@:::@@..::@@@.:::......:::.....::...:::::::"
echo " :.@@@@@@@@@-.-..@@@@@@@@::::::....::@@:::@@.:::.....::::@@@@@@@@....@@@@@@@@@:::"
echo " :::::::::::@@@@@.:::::::::::::..::::@@:::@@..::.....:::::::::::-@@@@.:::::::::::"
echo " ::::::::::@@.::@@:::::::::::::@@::::@@:::@@..::@@......:::::::*@...@@:::::::::::"
echo " :::::::::::@@*@@=:::::::::::::@@::::@@:::@@..::@@....::::::::::@@@@@@:::::::::::"
echo " ::::::::::::::::::+@*:::::::::@@::::@@:::@@..::@@...:::::::@@:::::::::::::::::::"
echo " :::::::::::::::::::::@@.::::::@@:.::@@:::@@.:::@@..:::::@@@:::::::::::::::::::::"
echo " :::::::::::::::::::::::#@@@.::@@:.::@@:::@@.:::@@..:+@@@::::::::::::::::::::::::"
echo " :::::::::::::::::::::::::::=@%@@:.::@@:::@@....@@+@@.:::::::::::::::::::::::::::"
echo " ::::::::::::::::::::::::::::::@@::::@@:::@@...:@@..:::::::::::::::::::::::::::::"
echo " ::::::::::::::::::::::::::::::@@::::@@:::@@..::@@.::::::::::::::::::::::::::::::"
echo " ::::::::::::::::::::::::::::::@@::::@@:::@@.:::@@.::::::::::::::::::::::::::::::"
echo " ::::::::::::::::::::::::::::::@@::::@@:::@@.:::@@.::::::::::::::::::::::::::::::"
echo " ::::::::::::::::::::::::::::::@@::::@@:::@@.:::@@.::::::::::::::::::::::::::::::"
echo " ::::::::::::::::::::::::::::::@@.:::@@:::@@.:::@@.::::::::::::::::::::::::::::::"
echo " ::::::::::::::::::::::::::::::@@::::@@:::@@.:::@@.::::::::::::::::::::::::::::::"
echo " ::::::::::::::::::::::::::::::@@::::@@:::@@.:::-@.::::::::::::::::::::::::::::::"
echo " ::::::::::::::::::::::::::::::::::::@@:::@@.::::::::::::::::::::::::::::::::::::"
echo " ::::::::::::::::::::::::::::::==::::@@:::@@.::::::::::::::::::::::::::::::::::::"
echo " ::::::::::::::::::::::::::::::::::::@@:::@@.::::::::::::::::::::::::::::::::::::"
echo " :::::::::::::::::::::::::::::::::::...:::...::::::::::::::::::::::::::::::::::::"
echo " :::::::::::::::::::::::::::::::::::.@@@@@@@@::::::::::::::::::::::::::::::::::::"
echo " ::::::::::::::::::::::::::::::::::::...:::::::::::::::::::::::::::::::::::::::::"
echo " :::::::::::::::::::::::::::::::::::.@@#.:@@..:::::::::::::::::::::::::::::::::::"
echo " :::::::::::::::::::::::::::::::::::::.@@@@...:::::::::::::::::::::::::::::::::::"
echo " ::::::::::::::::::::::::::::::::::::::.@@..:::::::::::::::::::::::::::::::::::::"
echo " ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::"
