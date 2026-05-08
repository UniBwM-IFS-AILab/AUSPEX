## Prepare the A Windows Desktop PC for AUSPEX
### 1. Prepare Linux Environment via WSL
1. Download and install WSL2, e.g. by the `wsl --install`command in PowerShell, as shown in https://learn.microsoft.com/en-us/windows/wsl/install. This will provide a virtual linux environment.

2. Before cloning this repository, prepare an Ubuntu 22.04 WSL instance.
   The easiest way is to download a tarball, which contains an [Ubuntu 22.04 image](https://cloud-images.ubuntu.com/wsl/releases/22.04/current/).
   To import the image to your WSL open a PowerShell terminal and run:
      ```
      wsl --import Dev_Companion22 <path-to-install> <path-to-tarball>
      ```
   It is important to set the distribution name to:
      ```
      Dev_Companion22
      ```
   The path-to-install can be chosen, e.g. `C:\WSL\devcompanion`.
   Before running the WSL instance add:
      ```
      [wsl2]
      networkingMode=mirrored
      ```
   to the WSL config in Windows (create one if not existing):
      ```
      C:\Users\YourUsername\.wslconfig
      ```
   This enables the mirroring of the Host Network. To ensure wsl is restarted type:
      ```
      wsl --shutdown
      ```
   Then run the new WSL instance by:
      ```
      wsl -d Dev_Companion22
      ```
   To add a new user (besides root):
      ```
      sudo adduser companion
      ```
   and add it to the sudo group
      ```
      adduser companion sudo
      ```
   switch to the new user by:
      ```
      su companion
      ```
   add this user as default user on startup and enable systemd (This will replace the wsl.conf file):
      ```
      sudo sh -c "echo '[user]\ndefault=$(whoami)\n[boot]\nsystemd=true' > /etc/wsl.conf"
      ```

3. Open Ports for PX4 in WSL2 (Windows)

   Open ports using the firewall configuration: run `WF.msc` ->  `inbound rules` -> `Action: New Rule` to open ports required for connecting PX4 running in WSL to the unreal simulation.
   - Add incoming `TCP port 4560-4570`
   - Add incoming `UDP port 14540-14550`

4. We are using either WSL2 on Windows or a native Ubuntu 24.04 to host our AUSPEX system, where ROS 2 and external components are installed inside a docker container.

    To get started, run one update and upgrade:
    ```
    sudo apt update && sudo apt upgrade
    ```
    to install docker follow [Docker Installation Guide](https://docs.docker.com/engine/install/ubuntu/) and give permissions:
    ```
    sudo usermod -aG docker $USER
   ```
Now you can set up your git configuration and begin the installation of AUSPEX.
