#!/bin/bash
# a script to set up the environment for the AUSPEX Framework
# pull the submodules

HOST_AUSPEX_DIR=$(dirname "$(realpath "$0")")

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

# check if variant is provided as command line argument
if [ -z "$1" ]; then
    echo "no argument provided. Please start script with either "TERRA" or "AVIS" as argument."
    exit 1
fi

AUSPEX_PLATFORM=$1

# check if variant is supported (TERRA or AVIS)
if [ $AUSPEX_PLATFORM != "TERRA" ] && [ $AUSPEX_PLATFORM != "AVIS" ]; then
    echo "The string "$AUSPEX_PLATFORM" is not supported. Please enter TERRA or AVIS."
    exit 1
fi

add_to_file_if_not_present "source $HOST_AUSPEX_DIR/utils/host_aliases.sh" ~/.bashrc
add_to_file_if_not_present "source $HOST_AUSPEX_DIR/AUSPEX-VASA/scripts/vasa_aliases.sh" ~/.bashrc

if [ ! -f "$HOST_AUSPEX_DIR/utils/user_exports.sh" ]; then
    echo "copying initial user_exports file."
    if [ $AUSPEX_PLATFORM == "TERRA" ]; then
        cp "$HOST_AUSPEX_DIR/utils/user_exports/user_exports_templ_terra.sh" "$HOST_AUSPEX_DIR/utils/user_exports.sh"
    else
        cp "$HOST_AUSPEX_DIR/utils/user_exports/user_exports_templ_avis.sh" "$HOST_AUSPEX_DIR/utils/user_exports.sh"
    fi

    sed -i "s|^export AUSPEX_PLATFORM=.*|export AUSPEX_PLATFORM=$AUSPEX_PLATFORM|" "$HOST_AUSPEX_DIR/utils/user_exports.sh"
else
    echo "user_exports already exists."
fi

# updating the submodules respective to the platform in use
if [ $AUSPEX_PLATFORM == "TERRA" ]; then
    echo "cloning repository for "down to earth""
    cd $HOST_AUSPEX_DIR && git submodule update --init --recursive AUSPEX-PLAN AUSPEX-EXEC AUSPEX-MSGS AUSPEX-KNOW AUSPEX-AERO AUSPEX-VASA

elif [ $AUSPEX_PLATFORM == "AVIS" ]; then
    echo "cloning repository for "in the sky""
    cd $HOST_AUSPEX_DIR && git submodule update --init AUSPEX-MSGS AUSPEX-AERO AUSPEX-VASA
    cd AUSPEX-AERO
    git submodule update --init --recursive
fi

cd $HOST_AUSPEX_DIR/AUSPEX-AERO
git submodule update --init --recursive
git checkout main > /dev/null 2>&1

# clone AUSPEX-VASA
cd $HOST_AUSPEX_DIR/AUSPEX-VASA
git checkout main > /dev/null 2>&1

# clone AUSPEX-MSGS
cd $HOST_AUSPEX_DIR/AUSPEX-MSGS
git submodule update --init --recursive
git checkout main > /dev/null 2>&1
cd src/UPF4ROS2
git sparse-checkout init --cone
git sparse-checkout set upf_msgs

# clone modules for TERRA variant
if [ $AUSPEX_PLATFORM == "TERRA" ]; then
    echo "setting up desktop environment..."

    # clone AUSPEX-SENS
    echo "installing(git): SENS ..."
    cd $HOST_AUSPEX_DIR/AUSPEX-SENS
    git submodule update --init --recursive
    git checkout main > /dev/null 2>&1

    # clone AUSPEX-PLAN
    echo "installing(git): PLAN ..."
    cd $HOST_AUSPEX_DIR/AUSPEX-PLAN
    git submodule update --init --recursive
    git checkout main > /dev/null 2>&1
    cd src/UPF4ROS2
    git sparse-checkout init --cone
    git sparse-checkout set upf4ros2

    # clone AUSPEX-EXEC
    echo "installing(git): EXEC ..."
    cd $HOST_AUSPEX_DIR/AUSPEX-EXEC
    git checkout main > /dev/null 2>&1

    # clone AUSPEX-KNOW
    echo "installing(git): KNOW ..."
    cd $HOST_AUSPEX_DIR/AUSPEX-KNOW
    git checkout main > /dev/null 2>&1
fi

# copy platform properties file
if ! [ -d "$HOME/auspex_params" ]; then
    mkdir $HOME/auspex_params >/dev/null
    cp -r $HOST_AUSPEX_DIR/utils/auspex_params $HOME
fi

echo ""
echo "Please update the ~/auspex_params/platform_properties/platform_properties.json and type 'setup_auspex'. Then Restart WSL (wsl --shutdown in powershell) or OBC"

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
