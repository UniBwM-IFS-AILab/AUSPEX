### A guide on how to set up real hardware with AUSPEX
---
# Setup Multikopter MK-U20
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
- Prepare Custom v1.15.4 Firmware with dds_conf.yaml
- Configure PX4 to start up with the correct UXRCE Client config. In the MavLink Console of QGC type and edit {name}:
    ```
    cd /fs/microsd/
    mkdir etc && cd etc
    "" >> config.txt
    echo "uxrce_dds_client stop" >> extras.txt
    echo "uxrce_dds_client start -t serial -d /dev/ttyS3 -b 921600 -n {name}" >> extras.txt
    ```
- In QGC set UXRCE_DDS_CFG2 to Telem2 and UXRCE_DDS_PTCFG to Custom Participant

## 2. Setup for Jetson Xavier NX

- Use the TTYTHS0 device to interface the Oragne Cube+:
```
MicroXRCEAgent serial --dev /dev/ttyTHS0 -b 921600 -r ~/dds_local_setup.xml
```
- Install auspex and vasa etc

---

# Setup Holybro X500 v2
## 1. Setup for the PixHawk 6c (PX4 v1.15.4)
- Prepare Custom v1.15.4 Firmware with dds_conf.yaml
    ```
    git clone -b v1.15.4 https://github.com/PX4/PX4-Autopilot.git
    cd PX4-Autopilot
    make px4_fmu-v6c_default
    ```
- Configure PX4 to start up with the correct UXRCE Client config. In the MavLink Console of QGC type and edit {name}:
    ```
    cd /fs/microsd/
    mkdir etc && cd etc
    "" >> config.txt
    echo "uxrce_dds_client stop" >> extras.txt
    echo "uxrce_dds_client start -t serial -d /dev/ttyS3 -b 921600 -n {name}" >> extras.txt
    ```
- In QGC set UXRCE_DDS_CFG to Telem2 and UXRCE_DDS_PTCFG to Custom Participant
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
    build_pi_vasa
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
        nmcli dev wifi list --rescan yes
        ```
        - and then add the Wi-Fi to your known networks:
        ```
        nmcli connection add \
        con-name    hummingbirdX_network \
        ssid        hummingbirdX_network \
        wifi-sec.psk password
        ```
        - if a connection should be established (Not necessary; after reboot, it will auto-connect if the network is available):
        ```
        nmcli connection up hummingbirdX_network
        ```