### Network Setup Wi-Fi (Only needed for controlling real hardware)
For setting up a Wi-Fi connection from your PC to your UAVs, you need to install some packages to enable [B.A.T.M.A.N mesh](https://www.open-mesh.org/doc/batman-adv/Quick-start-guide.html) running: `sudo apt install jq iw iproute2 batctl rfkill network-manager isc-dhcp-client wireless-tools`.

Go to `utils/wifi` folder to find the scripts, which you should run as root user. This will create a Wi-Fi mesh according to the definitions made in the `platform_properties.json` file you copied outside the repo earlier.

For performing network managements using `net_bench-sh` script, it is necessary running `sudo apt install iputils-ping fping iperf3 python3 iproute2` to install necessary packages.

Make enable, disable and startup scripts executable via `chmod +x`


To Enable Autostart, add a systemd service file at: 
`sudo nano /etc/systemd/system/batman-mesh.service`

With the following content:
```
[Unit]
Description=Batman-adv Mesh Network Setup
After=network-online.target
Wants=network-online.target

[Service]
Type=oneshot
User=root
ExecStart=/home/hummingbird6/AUSPEX/utils/wifi/startup_BATMAN_WiFi.sh
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

Enable to run the service on boot:
```
sudo systemctl daemon-reload
sudo systemctl enable batman-mesh.service
```

Check it by performing reboot.


### Clock Synchronization
Restarting the Raspberry PI (Node) without internet will lead it to use an outdated time, as it is not set to the current time.
To handle this, [chrony](https://chrony-project.org/) can be used which is synchronizing clocks between between the ground station (which is assumed to have the correct clock time) and the Nodes

- Install chrony using `sudo apt install chrony`
- Edit its settings by `sudo nano /etc/chrony/chrony.conf`
    - For the ground station only: 
        - add `local stratum 8` to allow the GCS to act as time source even without upstream servers.
        - add `allow 10.8.0.0/24` to allow chrony publish in the batman adv subnet.
    - For the nodes only: 
        - add `server 10.8.0.XX iburst prefer` with the IP of the time providing device. You can copy/paste this line for multiple devices / IPs.
    - On all devices connected with chrony:
        - adapt `makestep 1.0 3`
    - save the settings
- restart with `sudo systemctl restart chrony`

You can view the sources (interesting on the nodes) with `chronyc sources`, where it should show `^* 10.8.0.XX` as source. Check the offset with `chronyc tracking`
