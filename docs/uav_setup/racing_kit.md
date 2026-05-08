# Setup Racing Drone
The Racing Drones are custom build and come with the following hardware:
- MATEKSYS Flight Controller H743-SLIM V3 https://www.mateksys.com/?portfolio=h743-slim for Ardupilot https://ardupilot.org/copter/docs/common-matekh743-wing.html
- NVIDIA Jetson Orin NX (not the original Developer Kit) with 16 GB RAM
- FrSky ARCHER R4 Receiver https://www.frsky-rc.com/product/archer-r4/

Important: 
- The UAVs back is where the GPS receiver (covered by orange tape) is located. 
- The GPS Receiver LED is always lighting up green when powered. Once a valid position was found, the LED is flashing green


### 1.3 Connect Remote
- put batteries in the remote and connect it to the receiver according to the receiver manuals `Registration & Automatic binding` section https://www.frsky-rc.com/wp-content/uploads/Downloads/Manual/ARCHER%20R4/ARCHER%20R4%20-Manual.pdf


### 1.4 Calibrate Part 2 - Radio
![Overview of the remotes manual with added comments](Manual_FrSky_Taranis.jpg "Manual Remote")
- Calibrate the radio using the guided steps. At some point, you must have moved all sticks and switches to be able to assign them later.
- take the manual out of the box to get a better overview of the switches names
- Somehow, only the both sticks and their functions are occupied. You need to add all switches manually as written below.
- Go to the main menu (three horizontal bars) and follow to the `INPUTS` (page 5/12) by clicking on the PAGE button:
    - Select a free input, press ENTER (center of the wheel)
    - go to `Source` and press ENTER
    - move the input you would like to use (a shortcut will be displayed for the switch, like `xy`) and press EXIT
    - move to `Input` and assign the same shortcut there as displayed in `Source`
    - confirm this with exit and move back with exit as well
    - repeat this for all inputs. Now, you have added the inputs of the remote internally. We need to do that externally as well to be able to use it with the Holybro drone.
- move one page forward to `MIXES` (page 6/12):
    - Add the functions to the channels by going to a free Channel `CHX`
    - press ENTER to go the specific channel `CHX`
    - one line below at `Mix name`, you can assign the function name of the switch to it, confirm with EXIT (if a switch has no function, leave its name free)
    - select at `Source` the matching source. 
- moving now the sticks, knobs and switches, you can see that the sliders in QGroundControl moving.
It would be now the correct time to put stickers on the remote about what the switches are doing.
- In the next step, move to `Flight Modes` in QGroundControl to add assign the flight modes to the specific switches. See the table below for the allocation.

| Setting | Channel |
| ----------- | ----------- |
| Mode Channel | Ch7 |
| Flight Mode 1 | Takeoff |
| Flight Mode 4 | Position |
| Flight Mode 6| Land |
| Flight Mode 2,3,5 | Unassigned |
| Arm switch channel | Ch5 |
| Emergency Kill switch channel | Ch10 | 
| Offboard switch channel | Ch8 |
| Landing gear switch channel | Unassigned |
| Loiter switch channel | Unassigned |
| Return switch channel | Ch9 | 

For the values of the switches go from -100 to +100. The -100 is located away from the person, the +100 is located towards the person.


### 1.5 Calibrate Part 3 - Power, Actuators and Safety
- In the `Power Config` set the `Number of Cells (in Series)` to 6 and adjust the voltage from 3.70 to 4.2V (but check with the used batteries) and perform the `ESC OWM Minimum and Maximum Calibration`.

- In `Actuators Config` check the position of the motors and in `PWM MAIN` assign the MAIN 1 - 4 to the Motors 1 - 4. Now test the actuators carefully.

- Last, go to `Safety` and check the the Failsafe Flags in QGC:
    ```
    Set Low Battery Failsafe Trigger --> Failsafe Action to Land mode
    Set RC Loss Failsafe Trigger to Return mode
    Set Return to Launch Settings the Climb to altitude of 30m
    ```


### 1.6 Setup for the PixHawk 6c
In QGrouundControl go to `Parameters` and change the following settings. This is necessary to enable the connection of an external micro controller to the Flight Controller.

For mavsdk via QGroundControl (Important Telem3):
```
Set MAV_0_CONFIG to Telem3
Set MAV_0_MODE to OnBoard
Set MAV_0_RATE to 0
```
Reboot and set Baudrate:
```
Set SER_TEL3_XX to 921600
```