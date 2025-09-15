# How to connect to OpenVPN on boot

## Credentials
Edit *auth.txt* and enter username and password for your VPN

## OpenVPN client config
Add .ovpn client config file to this folder and match filename "client_config.ovpn".
Add auth.txt file if any credentials are required.
```
touch auth.txt
echo -e "USERNAME\nPASSWORD" >> auth.txt
```

## Make openvpn executable without password
Enter:
```
sudo visudo
```

Add this line at end of file:
```
your_username ALL=(ALL) NOPASSWD: /usr/sbin/openvpn
```

## Add cronjob for running script on boot up
Enter:
```
crontab -e
```

Add this line:
```
@reboot sleep 10 && /home/DRONENAME/AUSPEX/utils/ovpn/connect-vpn.sh
```

## Reboot
