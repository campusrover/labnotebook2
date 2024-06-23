---
title: Inspect Rasberry Pi Model
desc: Commands to inspect the version of the pi
status: current
author: Pito Salas
date: 23-jun-2024
---
# Figuring out Raspberry Pi Version

1. Use the `cat /proc/device-tree/model` command: If you have a Raspberry Pi OS installed and running, you can open a terminal and run the command cat /proc/device-tree/model. This will display the exact model name of your Raspberry Pi.
2.  Check the cpuinfo file: Another option is to run `cat /proc/cpuinfo` in the terminal. Look for the "model name" line, which will indicate the specific Raspberry Pi model.