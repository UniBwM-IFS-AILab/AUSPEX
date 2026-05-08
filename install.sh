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

add_to_file_if_not_present "source $HOST_AUSPEX_DIR/utils/host_aliases.sh" ~/.bashrc
add_to_file_if_not_present "source $HOST_AUSPEX_DIR/AUSPEX-VASA/scripts/vasa_aliases.sh" ~/.bashrc

if [ ! -f "$HOST_AUSPEX_DIR/utils/user_exports.sh" ]; then
    echo "copying initial user_exports file."
    cp "$HOST_AUSPEX_DIR/utils/user_exports_template/user_exports_template.sh" "$HOST_AUSPEX_DIR/utils/user_exports.sh"
else
    echo "user_exports already exists."
fi

# updating the submodules respective to the platform in use
cd $HOST_AUSPEX_DIR && git submodule update --init --recursive AUSPEX-PLAN AUSPEX-EXEC AUSPEX-MSGS AUSPEX-KNOW AUSPEX-AERO AUSPEX-VASA AUSPEX-SENS

cd $HOST_AUSPEX_DIR/AUSPEX-AERO
git checkout main > /dev/null 2>&1

# clone AUSPEX-VASA
cd $HOST_AUSPEX_DIR/AUSPEX-VASA
git checkout main > /dev/null 2>&1

# clone AUSPEX-MSGS
cd $HOST_AUSPEX_DIR/AUSPEX-MSGS
git checkout main > /dev/null 2>&1
cd src/UPF4ROS2
git sparse-checkout init --cone
git sparse-checkout set upf_msgs

# clone AUSPEX-KNOW
echo "installing(git): KNOW ..."
cd $HOST_AUSPEX_DIR/AUSPEX-KNOW
git checkout main > /dev/null 2>&1

# clone AUSPEX-SENS
echo "installing(git): SENS ..."
cd $HOST_AUSPEX_DIR/AUSPEX-SENS
git checkout main > /dev/null 2>&1

# clone AUSPEX-EXEC
echo "installing(git): EXEC ..."
cd $HOST_AUSPEX_DIR/AUSPEX-EXEC
git checkout main > /dev/null 2>&1

# clone AUSPEX-PLAN
echo "installing(git): PLAN ..."
cd $HOST_AUSPEX_DIR/AUSPEX-PLAN
git checkout main > /dev/null 2>&1
cd src/UPF4ROS2
git sparse-checkout init --cone
git sparse-checkout set upf4ros2

# initialize params directories from examples without overwriting existing data
PARAMS_DIR="$HOST_AUSPEX_DIR/params"
EXAMPLE_PARAMS_DIR="$PARAMS_DIR/examples"

for params_subdir in config geographic mission platform_properties; do
    if ! [ -d "$PARAMS_DIR/$params_subdir" ]; then
        if [ -d "$EXAMPLE_PARAMS_DIR/$params_subdir" ]; then
            cp -r "$EXAMPLE_PARAMS_DIR/$params_subdir" "$PARAMS_DIR/$params_subdir"
        elif [ -d "$HOST_AUSPEX_DIR/params/$params_subdir" ]; then
            cp -r "$HOST_AUSPEX_DIR/params/$params_subdir" "$PARAMS_DIR/$params_subdir"
        fi
    fi
done


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
echo ""
echo "Please update the $HOST_AUSPEX_DIR/params/platform_properties/platform_properties.yaml and type 'auspex_setup' and resource the shell."
