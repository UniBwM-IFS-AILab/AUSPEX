# Use Case 1: Search Mission

## Requirements:

Follow the general installation section for the Base Station in the main [AUSPEX](https://git.unibw.de/angewandte-ki-f-r-dynamische-systeme/AUSPEX) repositiory. Thereafter, follow the real uav setup section to prepare your UAV. Summed up, install AUSPEX as follows:
- **UAV:** AUSPEX needs to be installed in **AVIS** mode:
    ```
    cd ~/AUSPEX
    ./install.sh AVIS
    ```
    This installs a lightweight version of AUSPEX, including only the components necessary for controlling the UAV. The script may take a while to process.
- **Ground Station:** Install AUSPEX in **TERRA** mode:
    ```
    cd ~/AUSPEX
    ./install.sh TERRA
    ```
    This will install all components of AUSPEX.

After AUSPEX is installed on the UAV and the Base station, it is important to setup your network according to the Network Setup section in the AUSPEX main repository. Also do not forget to setup the ```platform_properties.json```.

## Running the Example:

To run use case 1 "Search Mission", follow the instructions below.

 1. First construct your problem and domain file. These files can be found in the base station AUSPEX installation in ```~/AUSPEX/AUSPEX-PLAN/src/auspex_planning/pddl/```. For defining PDDL files, you can look up [UP](https://unified-planning.readthedocs.io/en/latest/).
 2. Open the ```planner_config.json``` and enter ```pddl_planner``` to select this planner for your ```<team_id>``` (defined in the ```platform_properties.json```):
    ```
    [
        {
        "team_id": "drone_team",
        "planner": "pddl_planner",
        "params": []
        },
    ]
    ```
3. **UAV:**
    On the companion computer of the UAV start the following commands in a separate shell:
    - Start the docker container:
        ```
        run_avis_pi_vasa
        ```
    - And in a second terminal type the follwoing, to start the &mu;-XRCE-DDS-Agent:
        ```
        iv start_rtps_agent_drone
        ```
    - To start the offboard controller type in a third terminal:
        ```
        iv start_offboard_control
        ```
4. **GCS:**
    On the ground control station type ```run_terra_vasa``` to start the docker environment and in another shell run ```win_start_gcs```, to start the necessary commands each in its own shell tab.

    **Alternatively**, it is possible to start each command yourself, by using the following aliases in a separate shell in the given order. The alias ```iv``` means "in vasa" and starts the command in the docker container.<br><br>
    - Start the docker container:
        ```
        run_terra_vasa
        ```
    - Start the Copernicus server, which provids height data to the system:
        ```
        iv start_copernicus_server
        ```
    - Start ValKey as database:
        ```
        iv "stop_valkey && start_valkey"
        ```
    - Start the Knowledge Base, interfacing ValKey:
        ```
        iv start_knowledge_main
        ```
    - Start UP4ROS2:
        ```
        iv start_up4ros
        ```
    - Start the Executor Manager:
        ```
        iv start_executor_main
        ```
    - Start the Planning Main:
        ```
        iv start_planning_main
        ```
    - Start the perception module:
        ```
        iv start_perception_main
        ```
    The different modules can be deployed distributed and are not necessarily bound to the same hardware. After setting everything up, the last shell should display ```Planner Main ready...```

5.  To interact with the planner interface, start the CLI:
    ```
    start_command_publisher
    ```
    The available commands are displayed. To plan a mission type:
    ```
    plan <team_id>
    ```
    and hit enter. This will query the planner for ```<team_id>``` from the knowledge base, which will be the ```pddl_planner```. This planner will forward the domain and problem file, specified earlier, to UP and wait for a plan. When a plan is computed it is loaded into the knowledge base and can be accepted by sending the command:
    ```
    accept <team_id>
    ```
    This will notify the planner, to query the plan for ```<team_id>``` from the knowledge base and forward it to the respective executors.

After these 5 steps, the UAV should be flying and executing the computed plan form UP.

## Visualisation

To visualize the plan and the position of the UAV, the GUI can be started by:
```
cd AUGUR/
flutter run
```
Enter the IP address of your base station (in general it is set to 127.0.0.1 (localhost)) and ýou are ready to go.