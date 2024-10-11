---
title: Hostnames
author: Pito Salas
date: apr-2024
status: current
---
# Hostnames

All computers (`hosts`) on the network have (or can have) a hostname. This is nothing more that what you might think of as the name of the computer. In the case of our robots, each robot has a hostname, e.g. `platform3`.

## Setting hostnames

To change the hostname on an Ubuntu system, you need to modify it in several places. Here are the steps to change the hostname permanently:

1. Edit the `/etc/hostname` file:
   ```bash
   sudo nano /etc/hostname
   ```
   Replace the current hostname with the new one you want.

2. Update the `/etc/hosts` file:
   ```bash
   sudo nano /etc/hosts
   ```
   Find the line with the old hostname and update it to the new hostname.

3. Use the `hostnamectl` command to set the new hostname:
   ```bash
   sudo hostnamectl set-hostname new-hostname
   ```
   Replace `new-hostname` with your desired hostname.

4. To prevent cloud services from resetting the hostname, you may need to modify cloud-init configuration:
   ```bash
   sudo nano /etc/cloud/cloud.cfg
   ```
   Find the line `preserve_hostname: false` and change it to `preserve_hostname: true`.

5. Reboot your system to apply all changes:
   ```bash
   sudo reboot
   ```

