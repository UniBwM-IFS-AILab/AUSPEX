# AUSPEX Software Setup for all Platforms

### 1. Install the AUSPEX Framework (Terra Version)
Inside your host system, for cloning this repository run following commands:
   ```
   git clone https://github.com/UniBwM-IFS-AILab/AUSPEX.git
   ```
For installation we provide an installation script, which will:
   - Init and pull the submodules
   - Setup environment variables and aliases

It can be found in main folder.
   ```
   cd ~/AUSPEX/
   ```
To install the framework run:
   ```
   ./install.sh
   ```
This will clone the submodules and set some default settings.

After the installs script finished copy the contents of `~/AUSPEX/params/platform_properties/example_platform_properties_UnrealEngine.yaml` to `~/AUSPEX/params/platform_properties/platform_properties.yaml` or enter individual values, which specify your simulated platform. In this folder are different other example platform properties for different platforms, select the one most suiting to your use case and complete the values.
   ```
   cp ~/AUSPEX/params/platform_properties/example_platform_properties_UnrealEngine.yaml ~/AUSPEX/params/platform_properties/platform_properties.yaml
   ```
and type (You might need to install the yaml package: `sudo apt install python3-yaml`):
   ```
   cd ~/AUSPEX/ && python3 setup.py
   ```
to read in the variables from the platform properties file.
Restart you host system or open a new shell to source the env variables.
To build the docker container type:
   ```
   vasa_build
   ```
This may take some time.
After building the container, start it and build all ROS 2 packages:
   ```
   vasa_shell
   auspex_rebuild
   ```
- Edit openvpn configs in the `~/AUSPEX/utils/ovpn/` folder and follow the advices on the `README`
After it is finished, the installation is done and ready to use.



