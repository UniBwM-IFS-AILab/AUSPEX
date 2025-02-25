# AUSPEX

**A**utomated **U**nmanned Aerial Swarm **S**ystem for **P**lanning and **EX**ecution

In ancient Rome an auspex (latin for "one who observes the birds") was a priest who studied bird behavior (flight patterns, calls, or feeding habits) to determine the will of the gods.

## Overview

AUSPEX is a modular framework for developing and validating AI planning algorithms for UAVs.
It implements [REAP](https://github.com/UniBwM-IFS-AILab/REAP) as simulation environment.

### AUSPEX components
   * AUSPEX-AERO &rarr; Onboard Software, including the Offboard Controller
   * AUSPEX-EXEC &rarr; **EXEC**ution Module
   * AUSPEX-PLAN &rarr; **PLAN**nning Module
   * AUSPEX-SENS &rarr; **SENS**or Data Processing Module
   * AUSPEX-MSGS &rarr; **M**e**S**sa**G**e**S**
   * AUSPEX-KNOW &rarr; World **KNOW**ledge Base
   * utils &rarr; folder containing utils to setup the environment

### External components
   * AirSim
   * PX4 Autopilot
   * Micro-XRCE-DDS-Agent

## System requirements

We tested AUSPEX using the following development environment:
   * Windows 11
   * WSL2 instance (Ubuntu 22.04 LTS)

On Windows 11 side Unreal Engine (confirmed working version is 4.27.2) with AirSim plugin needs to be running for the visualization part. This repository contains the rest of the framework - planning framework, executor, offboard controller, message defintions, ... - which is running inside WSL. So first make sure to install UE4 with AirSim on Windows.

When working with Colosseum (AirSim fork from CodexLabsLLC) Unreal Engine 5.2 is supported as well. For Colosseum follow the same instructions as with AirSim but use the CodexLabsLLC repository.

### Install Unreal Engine with the AirSim Plugin ([REAP](https://github.com/UniBwM-IFS-AILab/REAP))

Follow the instructions under: https://microsoft.github.io/AirSim/build_windows/. The most up-to-date Version of Unreal Engine confirmed to be working is currently 4.27.2.

Furthermore, AirSim requires a settings file to operate (should be located under "%USERPROFILE%/Documents/AirSim"). Exchange the default settings.json file with the settings file provided in this [repository](https://github.com/UniBwM-IFS-AILab/AUSPEX/-/tree/main/utils/airsim_settings?ref_type=heads). For detailed information about AirSim settings see: https://microsoft.github.io/AirSim/settings/.

### Setup WSL

Before cloning this repository prepare an Ubuntu 22.04 WSL instance.
The easiest way is to download a tarball, which contains an [Ubuntu 22.04 image](https://cloud-images.ubuntu.com/wsl/releases/22.04/current/).
To import the image to your WSL open a PowerShell terminal and run:
```
wsl --import Dev_Companion22 <path-to-install> <path-to-tarball>
```
It is important to set the distribution name to:
```
Dev_Companion22
```
The path-to-install can be chosen, e.g. C:\WSL\devcompanion

Run the new WSL instance by:
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
run one update and upgrade:
```
sudo apt update && sudo apt upgrade
```
Install python3 pip:
```
sudo apt install python3-pip
```
Install [torch](https://pytorch.org/get-started/locally/) for your distribution:
```
pip3 install torch torchvision torchaudio
```
Check your install with:
```
python3 -c "import torch; print(torch.cuda.is_available())"
```
Now you can set up your git configuration, assign a fixed IP address to this wsl and begin the installation of AUSPEX.python -c "import torch; print(torch.cuda.is_available())"

#### Git configurations
First create a Personal Access Token. This can be used for authentication without prompting for password and username on each Gitlab API access. A manual can be found
[here](https://docs.gitlab.com/ee/user/profile/personal_access_tokens.html)

And allow git to store your credentials:
```
git config --global credential.helper store
```

#### Assign fix IP address to WSL

As the IP address of WSL might change after restarting the Windows machine, it can be useful to set up a static address via a powershell script - provided in this repository as wsl_static_ip.ps1 - that gets executed on WSL startup. 
Download the script provided in the [repository](https://github.com/UniBwM-IFS-AILab/AUSPEX/-/blob/main/utils/wsl_win_script/wsl_static_ip.ps1?ref_type=heads) and move it to C:\WSL\Scripts.

Furthermore make sure to allow incoming connections in Windows:

Incoming TCP port 4560 and incoming UDP port 14540 are required for connecting PX4 running in WSL to the unreal simulation. The ports can be opened using the firewall configuration (run WF.msc -> left click inbound rules -> action: new rule).



## Installation
Step into your prepared WSL instance. For cloning this repository run:
```
git clone https://<username>:<personal access token>@git.unibw.de/angewandte-ki-f-r-dynamische-systeme/AUSPEX.git
```
For installation we provide an installation script, which will:
   * Install dependencies
   * Get the submodules
   * Modify configs of external submodules
   * Setup enviroment variables and aliases
   * Build all submodules

It can be found in main folder.
```
cd AUSPEX/
```
For running the script provide an argument, which specifies the installation variant.
It can be:
   * TERRA: full installation, including modules needed for simulation
   * AVIS: installs only modules needed for deploying on real hardware (e.g. on UAV)

Assuming you would like to install the full framework including the simulation modules run:
```
./install.sh TERRA
```
This will take some time as the submodules are cloned and built.
At the end of the script you are prompted to assign an instance name to your WSL, you are free to choose here.
After that, the installation is done and ready to use.

## Real UAV Setup
  
To setup AUSPEX on a real UAV, use the **AVIS** keyword during installation.
```
cd ~/AUSPEX
./install.sh AVIS
```
Overwrite the default ```platform_properties.json``` for each UAV to suite its respective capabilities. (Camera FOV, ...)
```
nano ~/auspex_params/platform_properties/platform_properties.json
```
Now the general setup is done and you are ready to use a real UAV with AUSPEX


## Network Setup

When working with real UAVs, the UAVs and the AUSPEX system should be in the same local network (e.g. via Router, VPN or ROS 2 Bridges). Moreover, to use ROS 2 over a VPN a Discovery Server is necessary. Set up a discovery server and export its IP on the UAV, as well as every system which is part of AUSPEX:
``` 
export ROS_DISCOVERY_SERVER=<DISCOVERY_IP>:11811
```
For convenience, add the export commands to the ```~./bashrc```.
Auspex can be deployed on distributed hardware, as long as each system is connected to the Discovery Server.
 
**Optional:** If the discovery server is not working, it is possible to set up the ROS 2 communication as peer to peer, multicast network. For this export the FASTRTPS_DEFAULT_PROFILES_FILE:

```
export FASTRTPS_DEFAULT_PROFILES_FILE=~/AUSPEX/utils/dds_scripts/fastrtps_dds_tailscale/fastrtps_dds_setup.xml
```

and add a ```<locator><udpv4><address>"PEER_IP"</address></udpv4></locator>``` tag, for each peer in the system and exchange the ```PEER_IP``` with the IP address of the respective peer. 
**NOTE:** This will drastically increase the communication overhead, in comparison to [discovery servers](https://docs.ros.org/en/foxy/Tutorials/Advanced/Discovery-Server/Discovery-Server.html).