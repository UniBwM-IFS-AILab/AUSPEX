# Use Case 4: Multi-UAV Coordination

## Requirements:

Follow the general installation section for the Base Station in the main [AUSPEX](https://git.unibw.de/angewandte-ki-f-r-dynamische-systeme/AUSPEX) repositiory. Summed up, install AUSPEX as follows:
- **Ground Station:** Install AUSPEX in **TERRA** mode:
    ```
    cd ~/AUSPEX
    ./install.sh TERRA
    ```
    This will install all components of AUSPEX.

Ensure Unreal Engine and AirSim are also installed on your system. An exemplary AirSim settings file is given in ```~/AUSPEX/utils/airsim_settings/settings.json```. This file will load one UAV in the simulation environment, equipped with a camera. Change the GPS position in the settings file to the actual GPS position of the UAV in your simulation environment.

## Running the Example:

To run use case 4 "Multi-UAV Coordination", follow the instructions below.

1. First define the waypoints in the ```areas.json``` located at ```~/auspex_params/geographic/areas/```. Enter the latitude. longitude and altitude of each point to be visited.
2. Open the ```plan_config.json``` and enter ```mvrp_alns_planner``` to select this planner for your ```<team_id>``` (defined in the ```platform_properties.json```) and specify the objective function to be used (here: "shortest_distance"):
    ```
    [
        {
        "team_id": "drone_team",
        "planner": "mvrp_alns_planner",
        "params": ["shortest_distance"]
        },
    ]
    ```
3. **Unreal Engine:** Start the simulation and deploy multiple UAVs (Adjust AirSim settings to spawn multple vehicles).
4. **GCS:**
    On the ground control station type ```runvasa``` to start the docker environment and in another shell run ```win_start_sim <number_of_uavs>```, to start the necessary commands each in its own shell tab for one UAV. by changing the ```<number_of_uavs>``` argument, multiple UAVs can be launched.

    **Alternatively**, it is possible to start each command yourself, by using the following aliases in a separate shell in the given order. The alias ```iv``` means "in vasa" and starts the command in the docker container.<br><br>
    - Start the docker container:
        ```
        runvasa
        ```
    - Start the flight controller software for each drone:
        ```
        iv "run_px4 <number_of_uavs>"
        ```
    - Start the offboard control node for each drone:
        ```
        iv "run_aero count:=<number_of_uavs>"
        ```
    - Start the Copernicus server, which provids height data to the system:
        ```
        iv run_copernicus
        ```
    - Start ValKey as database:
        ```
        iv "stop_valkey && run_valkey"
        ```
    - Start the Knowledge Base, interfacing ValKey:
        ```
        iv run_know
        ```
    - Start the Executor Manager:
        ```
        iv run_exec
        ```
    - Start the Planning Main:
        ```
        iv run_plan
        ```
    - Start the perception module:
        ```
        iv run_sens
        ```
    The different modules can be deployed distributed and are not necessarily bound to the same hardware. After setting everything up, the last shell should display ```Planner Main ready...```

5.  To start the planner, the command line is used. First start the command line interface with:
    ```
    iv run_cmd
    ```
    The available commands are displayed. To plan a mission type:
    ```
    plan <team_id>
    ```
    and hit enter. This will query the planner for ```<team_id>``` from the knowledge base, which will be the ```mvrp_alns_planner``` and plan a respective multi vehicle routing solution for the given area. When a plan is computed it is loaded into the knowledge base and can be accepted by sending the command:
    ```
    accept <team_id>
    ```
    This will notify the planner, to query the plan for ```<team_id>``` from the knowledge base and forward it to the respective executors.

After these 5 steps, the UAV should be flying and executing the computed plan.

## Visualisation

To visualize the plan and the position of the UAV, the GUI can be started by:
```
cd AUGUR/
flutter run
```
Enter the IP address of your base station (in general it is set to 127.0.0.1 (localhost)) and ýou are ready to go.