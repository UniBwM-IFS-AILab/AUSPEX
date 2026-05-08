## Prepare the Nvidia Jetson Xavier NX for AUSPEX: 
1. setup wifi
```
sudo apt update
sudo apt install dkms build-essential linux-firmware backport-iwlwifi-dkms
printf "iwlwifi\niwlmvm\n" | sudo tee /etc/modules-load.d/intel-wifi.conf
sudo depmod -a
sudo reboot
```
after reboot:
```
sudo nmcli device wifi connect "HuMiCS Lab" password "password" ifname wlP1p1s0
```
2. install nano: 
```sudo apt install nano curl```

3. Set up docker

For Xavier NX todo
```
sudo apt update
sudo apt install docker.io=27.5.1-0ubuntu3~22.04.2
sudo usermod -aG docker $USER
```

Log out log in
```
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
```
```
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
```

```
sudo apt update
sudo apt install nvidia-container-toolkit
```
```
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```
- Make sure your Jetson and machine are in the same network
- Connect to the Jetsonvia SSH by `username@hostname`and your password (if it does not work, WiFi might be off) and run the following commands

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