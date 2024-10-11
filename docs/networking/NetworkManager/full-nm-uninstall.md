---
title: Full NetworkManager Uninstall
description: How to completely uninstall NetworkManager
author: Pito Salas
updated: Oct 2024
status: draft
---
To completely uninstall NetworkManager and switch to using Netplan instead, follow these steps:

1. Stop and disable NetworkManager:

   ```bash
   sudo systemctl stop NetworkManager
   sudo systemctl disable NetworkManager
   ```

2. Uninstall NetworkManager and related packages:

   ```bash
   sudo apt purge network-manager network-manager-gnome network-manager-pptp network-manager-openvpn
   ```

3. Remove any leftover configuration files:

   ```bash
   sudo rm -rf /etc/NetworkManager
   ```

4. Update the package list:

   ```bash
   sudo apt update
   ```

5. Install netplan if it's not already installed:

   ```bash
   sudo apt install netplan.io
   ```

6. Create a new Netplan configuration file:

   ```bash
   sudo nano /etc/netplan/01-netcfg.yaml
   ```

7. Add your network configuration to the file. THis is the one we use«««

   ```yaml

network:
    ethernets:
        eth0:
            dhcp4: true
            optional: true
    version: 2
    wifis:
        wlan0:
            access-points:
                eduroam:
                    auth:
                        identity: "<robotics@brandeis.edu>"
                        key-management: "eap"
                        method: "peap"
                        password: "bapor-kibra"
                        phase2-auth: "MSCHAPV2"
                "one boston":
                    password: "ffabcd4444"
            dhcp4: true
            optional: true

   ```

8. Apply the netplan configuration:
   ```bash
   sudo netplan apply
   ```

9. Reboot your system to ensure all changes take effect:

   ```bash
   sudo reboot
   ```

After these steps, NetworkManager will be completely uninstalled and your system will be using netplan for network configuration.
