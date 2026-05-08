# Setup Multikopter MK-U20 (Ardupilot)

## 1. Hardware Setup

## 1.2. Setup for the Orange Cube Plus (PX4 v1.15.4)
- Start QGC on the remote controller.
    - Check the RC Settings:
        - MODE (Flight Mode) From farthest away from control person to closest.
            - Takeoff
            - Position
            - Land
        - SE (OBC Mode)
            - Off
            - Off
            - On
        - RTH (RTH)
            - Off
            - Off
            - On
        - AUTO (Kill Switch)
            - Off
            - Off
            - On

- For mavsdk via GQC:
    ```
    set MAV_0_CONFIG to Telem2
    Set MAV_0_MODE to OnBoard
    Set MAV_0_RATE to 0
    Set SER_Tel2_Baud to 57600
    ```
    maybe enable MAV_1_CONFIG for USB interface.
- The TTYTHS0 device is used to interface the Orange Cube+.

## 2. Setup for Jetson Orin NX/Xavier NX

### 2.0 Flashing Jetsons:
https://github.com/airvolute/dcs-deploy?tab=readme-ov-file#known-limitation---jetpack-62---beta
### 2.1 Building
You need: 
- A Jetson Orin NX/Xavier NX

### 2.2 Software Setup

#### 2.2.2. Connect to the Jetson and make adaptions 
1. set hostname
`sudo hostnamectl set-hostname new-hostname`
2. set user and password:
```
sudo adduser renameadmin
sudo adduser renameadmin sudo
exit
ssh renameadmin@address
sudo deluser --remove-home dcs_user
sudo adduser milan1
set password to dronesim
sudo adduser milan1 sudo

exit
ssh milan1@address
sudo deluser --remove-home renameadmin
```