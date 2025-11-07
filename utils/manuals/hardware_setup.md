### A guide on how to set up real hardware with AUSPEX
---
# Setup Multikopter MK-U20 (Ardupilot)
## 1. Hardware Setup

## 2. Setup for the Orange Cube Plus (PX4 v1.15.4)

- Install QGroundControl and connect to UAV
- Start QGC and calibrate.
    - For this it is possible to use the telemetry antennas to avoid the cable
    - RC Settings currently:
        - MODE (Flight Mode) From farthest away from control person to closest.
            - OBC
            - Altitude
            - Position
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

## 3. Setup for Jetson Xavier NX

- The TTYTHS0 device is used to interface the Orange Cube+.

- Install AUSPEX from the [Repo](https://github.com/UniBwM-IFS-AILab/AUSPEX/tree/main?tab=readme-ov-file) and follow the steps there

---

# Setup Holybro X500 v2 (PX4)

## 1. Hardware Setup

## 2. Setup for the PixHawk 6c (PX4 v1.15.4)

- For mavsdk via GQC:
    ```
    set MAV_0_CONFIG to Telem2
    Set MAV_0_MODE to OnBoard
    Set MAV_0_RATE to 0
    Set SER_Tel2_Baud to 921650
    ```
- Also set MAV_0_CONFIG to TELEM 1 to enable SIK Holybro Telemetry
- Check Failsafe Flags in QGC

## 3. Setup for the Raspberry PI 5

- Install supported OS
- update current packages:
    ```
    sudo apt update && sudo apt upgrade
    ```
- in the raspi-config enable the Serial Port:
    ```
    sudo raspi-config
    ```
    - Go to the Interface Option and then click Serial Port.
    - Select No to disable serial login shell.
    - Select Yes to enable the serial interface.
    - Click Finish and restart the RPi.
- edit /boot/firmware/config.txt and add this at the [all] tag at the end of the file:
    ```
    start_x=1
    gpu_mem=1024
    enable_uart=1
    dtoverlay=disable-bt
    dtparam=uart0
    ```
- to delete all default folder do:
    ```
    rm -r ~/Documents ~/Downloads ~/Music ~/Pictures ~/Public ~/Templates ~/Videos ~/Desktop ~/Bookshelf
    ```
- and set all paths to "$HOME" in:
    ```
    nano ~/.config/user-dirs.dirs
    ```
- install openvpn
    ```
    sudo apt install openvpn
    ```
- install docker on Raspbian OS 64 Bit
    ```
    curl -fsSL https://get.docker.com -o get-docker.sh
    sudo sh get-docker.sh
    sudo usermod -aG docker $USER
    sudo reboot
    ```
- install AUSPEX:
    ```
    git clone https://git.unibw.de/angewandte-ki-f-r-dynamische-systeme/AUSPEX.git
    cd AUSPEX/ && ./install.sh AVIS
    sudo reboot
    ```
- build docker container for avis (avis-vasa):
    ```
    buildvasa
    ```
- Edit openvpn configs in the ~/AUSPEX/utils/ovpn/ folder

- Connect to Wi-Fi on RPI
    - On RPI
        - first refresh your Wi-Fi list:
        ```
        sudo nmcli device wifi rescan
        sudo nmcli device wifi list
        ```
        - and then add the Wi-Fi to your known networks:
        ```
        sudo nmcli connection add type wifi ifname '*' con-name <name> ssid "<ssid>" wifi-sec.key-mgmt wpa-psk wifi-sec.psk "<password>" connection.autoconnect no
        ```
        - if a connection should be established (Not necessary; after reboot, it will auto-connect if the network is available):
        ```
        sudo nmcli connection up <ssid>
        ```