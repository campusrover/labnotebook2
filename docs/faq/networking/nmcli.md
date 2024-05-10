---
title: Network Manager and NMCLI
author: Pito Salas
updated: may-2024
---

## Using NMCLI
 
`nmcli` is a command line tool used for networking on Linux machines. It is typically installed by default on Ubuntu Linux, so you should not need to connect your machine through 
 a wired connection and install it before using it. However, if `nmcli` is not installed, follow the installation instructions.
 
 ### Installation (If needed)
 
 Connect your machine through a wired connection (e.g. To a router/wall connection through an ethernet port) and the following commands to install `nmcli` and configure it to
 start upon bootup of your machine.

 Note: If it doesn't work even with a connection, and you are on campus, send an email to help@brandeis.edu noting the mac address of the `eth0` device (using `ip link show`) and the fact that you are trying to connect. Ask them if they see any activity at that particular mac address.
 
 - `sudo apt-get update`
 - `sudo apt install network-manager`
 - `systemctl start NetworkManager.service`
 - `systemctl enable NetworkManager.service`

#### Make sure that networkd is disabled

**I am not an expert in ubuntu networking. ** There are two different and sort of compatible/incompatible network management stacks, known sometimesa as NeworkManager and networkd. I have found that they fight with each other. I try to make sure that networkd is totally turned off (which is suprisingly difficult!)

```
sudo systemctl stop systemd-networkd.service
sudo systemctl disable systemd-networkd.service
# more needed, have to look them up.
```

* Problems arise because there are two or three network management schemes on Ubuntu. There's `networkd` and there's `network-manager`. And they interact in very obscure ways. My current model (which remains to be proven) is to try to disable fully networkd and use only network-manager.

* `nmcli` is the cli for network-manager.
* `nmcli -t -f active,ssid dev wifi`  to find out what SSID I am connected over wifi
* `sudo nmcli dev wifi` list all wifi SSIDs visible
* `nmcli connection show` to show all connections that nmcli knows about
* `nmtui` for a textui to nmcli (networkmanager)


### Connections

* Viewing Connections and Their Priorities: `nmcli connection show`
* View specific details of a connection, including its autoconnect priority:: `nmcli connection show <connection-name>`
* Set priority of a connection: `nmcli connection modify <connection-name> connection.autoconnect-priority <priority-value>`
    * <connection-name>: The name of the connection profile you want to modify.
    * <priority-value>: An integer value representing the priority. Higher values have higher priority.
* Changing the name of a connection: `nmcli connection modify <current-name> connection.id <new-name>`
* Showing a list of connections with specific properties: `nmcli -f NAME,UUID,TYPE,DEVICE,STATE con show`
* Change autoconnect property: `nmcli connection modify <connection-name> connection.autoconnect yes`
