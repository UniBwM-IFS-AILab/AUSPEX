# Isaac Sim Setup and Integration with AUSPEX
[Nvidia's Isaac Sim](https://developer.nvidia.com/isaac/sim) serves as a simulation platform for visualization. To deploy UAVs in Isaac Sim we use the [Pegasus Simulator](https://pegasussimulator.github.io/PegasusSimulator/), which starts PX4 for each running UAV.

Utilized System:
- Workstation with high-end CPU, high-end Nvidia GPU and 128 GB RAM.
- Ubuntu 24.04.3 LTS
- Nvidia Driver Version 580.95.05
- Isaac Sim 5.1
- PX4-Autopilot v.14.3

## Preparation
1. Prepare IOMMU Issues: You need to deactivate IOMMU in the BIOS and in GRUB (Bootloader), as Isaac Sim uses CUDA and NVIDIA drivers, which do not support IOMMU-enabled PCIe peer-to-peer memory copy on bare-metal Linux.
    - Disable IOMMU in the BIOS
    - Disable IOMMU in GRUB
    - Verify 1: `sudo dmesg | grep -i iommu` should not give you outputs
    - Veryfy 2: `sudo find /sys/kernel/iommu_groups/ -type l` should not give you outputs

2. Update your Ubuntu:
    ```
    sudo apt update
    sudo apt upgrade
    ```
3. Download and setup [Git](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git)

## Install Isaac Sim
Install the above mentioned version of Isaac Sim for Linux(x86_64) as written in the top lines of [Pegasus Installation Guide](https://pegasussimulator.github.io/PegasusSimulator/source/setup/installation.html#installing-nvidia-isaac-sim), replacing the start of the installation video.

Once the Isaac Sim Application Selector window pops up, select `Start`. You can conduct a test as written in the [Isaac Sim Documentation](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/quick-install.html)


## Configure the environment variables
Continue as written in the [Pegasus Simulation Installation Tutorial](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/quick-install.html)

Hint: the `.bashrc` file can be found in the home directory.
Edit it with a command line editor like NANO and then source it again using `source .bashrc`.

ChatGPT told me  that checking if the simulator can be launched from a python script (standalone mode) is not working as they provided zsh commands and and gave new bash commands, which should be working:
```
${ISAACSIM_PYTHON} -c "print('Hello World')"

${ISAACSIM_PYTHON} ${ISAACSIM_PATH}/standalone_examples/api/isaacsim.core.api/add_cubes.py
```
Edit it with a command line editor like NANO or VIM and then source it again with `source .bashrc` command.


## Install Pegasus Simulation
Follow the installation tutorial for the [Pegasus Simulator](https://pegasussimulator.github.io/PegasusSimulator/source/setup/installation.html#installing-the-pegasus-simulator)

Here the updated command for bash:
```
${ISAACSIM_PYTHON} -m pip install --editable pegasus.simulator
```


## Install PX4-Autopilot
To install PX4-Autopilot using the `pip install` command on our used Ubuntu, we need a virtual python environment. Open a new terminal window and paste the following code:
To install PX4-Autopilot, using the `pip install` command on your used Ubuntu, we need a virtual python environment. Open a new terminal window and paste the following code:

```
cd ~/PegasusSimulator

# Create a venv just for PX4 tooling
python3 -m venv .px4_venv

# Activate it
source .px4_venv/bin/activate

# (Optional but recommended)
pip install --upgrade pip
```
At the start of the line, `(.px4_venv) user@PC` should be stated now.

Follow the installation steps for [PX-4 Autopilot](https://pegasussimulator.github.io/PegasusSimulator/source/setup/installation.html#installing-px4-autopilot) in the virtual python environment. Also, set the PX4 path inside the Pegasus Simulator to the created installation location.

From now on: whenever you want to build or work with PX4, first do:
`cd ~/PegasusSimulator && source .px4_venv/bin/activate`


## Tutorial
Run this [tutorial](https://pegasussimulator.github.io/PegasusSimulator/source/tutorials/run_extension_mode.html) to see if everything works together with [QGroundControl](https://qgroundcontrol.com/)


## Install AUSPEX
To enable Isaac Sim with Pegasus Simulator talking to AUSPEX,
you first need to install the `zmq` package via:
```
ISAACSIM_PYTHON -m pip install zmq
```

Afterwards, you need to add our [AUSPEX IsaacSim-Plugin](https://git.unibw.de/angewandte-ki-f-r-dynamische-systeme/auspex-isaac).
Installation is fairly simple and described in the readme. Do not forget to set your correct PX4-Path.

Now, you should be able to launch Isaac Sim with the command 'load_is_world', automatically loading the our testing environment as world. 
Hit the play button, you should see one/multiple drones sporning. Control is possible via QGroundControl or AUSPEX with AUGUR.