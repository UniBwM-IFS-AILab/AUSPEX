## Prepare the Raspberry Pi 5 for AUSPEX: 
### Software Setup
#### Install supported OS for the Raspberry Pi on the SD Card
- Go to https://www.raspberrypi.com/software/ and download the Raspberry Pi Imager for your machine. This allows to put the Raspberry Pi OS on a SD Card that can be inserted in the Raspberry Pi.

- Click through the Raspberry Pi Imager and select the device according to your hardware, the operating system as `Raspberry PI OS (64-BIT)` and put it on the SD Card

- After Clicking next, Imager will ask you if you would like to apply OS customization settings. Go through the pages and set up as following:
    - General --> Set hostname 
    - General --> Set username and password
    - General --> Configure wireless LAN
    - Services --> Enable SSH

- Write the data on the SD Card.

- Plug the SD Card in the Raspberry Pi.


#### Connect to the Raspberry Pi and make adaptions 

- Make sure your Raspberry Pi and machine are in the same network

- Connect to the Raspberry Pi via SSH by `username@hostname`and your password (if it does not work, WiFi might be off) and run the following commands

- update current packages:
    ```
    sudo apt update && sudo apt upgrade
    ```
- in the raspi-config enable the Serial Port:
    ```
    sudo raspi-config
    ```
    - Go to the `Interface Option` and then click `Serial Port`.
    - Select `No` to disable serial login shell.
    - Select `Yes` to enable the serial interface.
    - Click `Finish` and restart the RPi.
- edit `/boot/firmware/config.txt` and add this at the `[all]` tag at the end of the file:
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
- and set all paths to `$HOME` in:
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

- In case you did not connect to Wi-Fi during initial startup, you can do so now on the RPI
    - first refresh your Wi-Fi list:
        ```
        sudo nmcli device wifi rescan
        sudo nmcli device wifi list
        ```