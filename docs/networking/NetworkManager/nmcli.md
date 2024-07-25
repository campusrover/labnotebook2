---
title: Network Manager and NMCLI
description: Everything we know about NMCLI
author: Pito Salas
updated: may-2024
status: dated
---
!!! Important
    We don't use nmcli on the Brandeis Robots. This is informational only

# Using NMCLI
 
`nmcli` is a command line tool used for networking on Linux machines. It is typically installed by default on Ubuntu Linux, so you should not need to connect your machine through 
 a wired connection and install it before using it. However, if `nmcli` is not installed, follow the installation instructions.
 
### Installation (If needed)
 
 Connect your machine through a wired connection (e.g. To a router/wall connection through an ethernet port) and the following commands to install `nmcli` and configure it to start upon bootup of your machine.

 !!! Note: 
     If it doesn't work even with a connection, and you are on campus, send an email to help@brandeis.edu noting the mac address of the `eth0` device (using `ip link show`) and the fact that you are trying to connect. Ask them if they see any activity at that particular mac address.
 
 ```bash title="Install and run network manager"
 sudo apt-get update
 sudo apt install network-manager
 systemctl start NetworkManager.service
 systemctl enable NetworkManager.service
 ```

``` title="Make sure networkd is disabled"
sudo systemctl stop systemd-networkd.service
sudo systemctl disable systemd-networkd.service
# more needed, have to look them up.
```

!!! Note
    I am not an expert in ubuntu networking! There are two different and sort of compatible/incompatible network management stacks, known sometimesa as NeworkManager and networkd. I have found that they fight with each other. I try to make sure that networkd is totally turned off (which is suprisingly difficult!) Problems arise because there are two or three network management schemes on Ubuntu. There's `networkd` and there's `network-manager`. And they interact in very obscure ways. My current model (which remains to be proven) is to try to disable fully networkd and use only network-manager.

#### Cheat Sheet

```title="nmcli cheat sheet"
nmcli                                                           # is the cli for network-manager.
nmcli general                                                   # Summary status
nmcli device wifi connect "ssid" password "password"            # Set up a simple wifi connection
nmcli -t -f active,ssid dev wifi                                # to find out what SSID I am connected over wifi
sudo nmcli dev wifi                                             # list all wifi SSIDs visible
nmcli connection show                                           # to show all connections that nmcli knows about
nmcli connection show <connection-name>`                        # View specific details of a connection, including its autoconnect priority
nmcli conn del <UUID>                                           # Delete a connection
nmcli conn mod <current-name> connection.id <new-name>          # Changing the name of a connection:
nmcli -f NAME,UUID,TYPE,DEVICE,STATE con show                   # Showing a list of connections with specific properties
nmcli connection modify <connection-name> \
        connection.autoconnect yes                              # Change autoconnect property
nmcli connection modify <connection-name> \
        connection.autoconnect-priority <priority-value>        # Set priority of a connection
        # <connection-name>: The name of the connection profile you want to modify.
        # <priority-value>: An integer value representing the priority. Higher values have higher priority.
nmtui                                                           # for a textui to nmcli (networkmanager)
```
