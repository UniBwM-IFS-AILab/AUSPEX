### A guide on how to set up real hardware with AUSPEX
---
# Setup Multikopter MK-U20 (Ardupilot)
## 1. Setup for the Orange Cube Plus (PX4 v1.15.4)
- Prepare px4 version v1.15.4 with cubepilot_cubeorangeplus_default and custom:
    ```
    git clone -b v1.15.4 https://github.com/PX4/PX4-Autopilot.git
    cd PX4-Autopilot
    make cubepilot_cubeorangeplus_default
    ```
- Copy the PX4-Autopilot/build/cubepilot_cubeorangeplus_default folder to your host system and prepare the .px4 file.
- Install Mission planner and Upload firmware
    - Select Custom Firmware on Mission planner and select the .px4 file from the previous step.
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

## 2. Setup for Jetson Xavier NX

- The TTYTHS0 device is used to interface the Orange Cube+.

- Install auspex and vasa etc.

---

# Setup Holybro X500 v2 (PX4)
## 1. Setup for the PixHawk 6c (PX4 v1.15.4)
- Prepare Custom v1.15.4 Firmware with dds_conf.yaml
    ```
    git clone -b v1.15.4 https://github.com/PX4/PX4-Autopilot.git
    cd PX4-Autopilot
    make px4_fmu-v6c_default
    ```
- For mavsdk via GQC:
    ```
    set MAV_0_CONFIG to Telem2
    Set MAV_0_MODE to OnBoard
    Set MAV_0_RATE to 0
    Set SER_Tel2_Baud to 921650
    ```
- Also set MAV_0_CONFIG to TELEM 1 to enable SIK Holybro Telemetry
- Check Failsafe Flags in QGC

## 2. Setup for the Raspberry PI 5
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
- Clone Menthon-WS
- Create openvpn folder and add vpn configs
    ```
    mkdir ~/openvpn
    # add configs as x.client.ovpn
    ```
- Connect to USB Sim Card Network
    - Set up USB-Modem
        - Connect USB-Modem to a PC and connect to its Wi-Fi (password written on the USB-Stick)
        - Type 192.168.0.1 into your browser and set up Wi-Fi name to hummingbirdX_network with a password
    - On RPI
        - first refresh your Wi-Fi list:
        ```
        sudo nmcli device wifi rescan
        sudo nmcli device wifi list
        ```
        - and then add the Wi-Fi to your known networks:
        ```
        sudo nmcli connection add type wifi ifname '*' con-name hummingbirdX_network ssid "hummingbirdX_network" wifi-sec.key-mgmt wpa-psk wifi-sec.psk "password" connection.autoconnect no
        ```
        - if a connection should be established (Not necessary; after reboot, it will auto-connect if the network is available):
        ```
        sudo nmcli connection up hummingbirdX_network
        ```