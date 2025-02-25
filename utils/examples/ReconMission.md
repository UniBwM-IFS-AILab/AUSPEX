# Use Case 2: Reconnaissance Mission

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

To run use case 2 "Reconnaissance Mission", follow the instructions below.
 
1. First define the search area in the ```example_mission.json``` located at ```~/auspex_params/mission/```. Enter the description and at least three GPS points to the list, spanning the search area. Additionally, complete the rest of the file, with the ```<team_id>```, no-fly zones, and other constraints. 
2. Open the ```planner_config.json``` and enter ```pattern_planner``` to select this planner for your ```<team_id>``` (defined in the ```platform_properties.json```):
    ```
    [
        {
        "team_id": "drone_team",
        "planner": "pattern_planner",
        "params": []
        },
    ]
    ```
3. **Unreal Engine:** Start the simulation and deploy an UAV.
4. **GCS:** 
    On the ground control station type ```start_simulation <number_of_uavs>```, to start the necessary commands each in its own shell tab for one UAV. by changing the ```<number_of_uavs>``` argument, multiple UAVs can be launched.

    **Alternatively**, it is possible to start each command yourself, by using the following aliases in a separate shell in the given order. <br><br>

    - Start the flight controller software for each drone:
        ```
        start_px4_multiple <number_of_uavs>
        ```

    - Start the &mu;-XRCE-DDS-Agent for each drone:
        ```
        start_rtps_agent <number_of_uavs>
        ```
    
    - Start the offboard control node for each drone:
        ```
        start_offboard_control count:=<number_of_uavs>
        ```

    - Start the Copernicus server, which provids height data to the system:
        ```
        start_copernicus_server
        ```
    - Start ValKey as database: 
        ```
        stop_valkey && start_valkey
        ```
    - Start the Knowledge Base, interfacing ValKey: 
        ```
        start_knowledge_main
        ```
    - Start the interface to Unified Planning: 
        ```
        start_up4ros
        ```
    - Start the Executor Manager: 
        ```
        start_executor_main
        ```
    - Start the Planning Main: 
        ```
        start_planning_main
        ```
    - Start the perception module: 
        ```
        start_perception_main
        ```
    The different modules can be deployed distributed and are not necessarily bound to the same hardware. After setting everything up, the last shell should display ```Planner Main ready...```

5.  To start the planner, the command line is used. First publish your example mission:
    ```
    ros2 run auspex_knowledge search_mission_mock_publisher
    ```
    and then start the command line interface with:
    ```
    start_command_publisher
    ```
    The available commands are displayed. To plan a mission type:
    ```
    plan <team_id>
    ``` 
    and hit enter. This will query the planner for ```<team_id>``` from the knowledge base, which will be the ```pattern_planner``` and plan a respective search pattern for the given area. When a plan is computed it is loaded into the knowledge base and can be accepted by sending the command:
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