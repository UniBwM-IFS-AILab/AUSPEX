### Network Setup VPN (Only needed for controlling real hardware)
For setting up a VPN connection from your PC to your UAVs, you need to install the OpenVPN open source community edition https://openvpn.net/community/ on your local machine.

-  Credentials: 
   edit `auth.txt` and enter username and password for your VPN

-  OpenVPN client config:
   Add `.ovpn` client config file to this folder and match filename `client_config.ovpn`.

   Add `auth.txt` file if any credentials are required.
   ```
   touch auth.txt
   echo -e "USERNAME\nPASSWORD" >> auth.txt
   ```

-  Make openvpn executable without password
   ```
   sudo visudo
   ```

-  Add this line at end of file:
   ```
   your_username ALL=(ALL) NOPASSWD: /usr/sbin/openvpn
   ```

-  Add cronjob for running script on boot up
   ```
   crontab -e
   ```

-  Add this line to delay the start of the VPN after reboot:
   ```
   @reboot sleep 10 && /home/<DRONENAME>/AUSPEX/utils/ovpn/connect-vpn.sh
   ```

-  Reboot
