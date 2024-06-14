---
title: Udev Introduction
author: Pito Salas
description: Udev is a complicated mechanism
status: obsolete|new|tophit|dated
created: 14-jun-2024
version: 1
---
# Udev
The name udev is an umbrella for a collection of concepts and tools that are used to give various serial devices a name. For example for linorobot there are two serial devices that we rely on: the teensy, which is connected by a USB and the Lidar which is connected to another USB. 

It would be convenient if we could refer to them by name. As is the case with the Linorobot.org package, we refer to them as `/dev/linobase` and `/dev/linolidar`

The udev mechanism ensures that these two "arbitrary" USB connections get those names, *no matter which USB port you plug them into".

This is low level stuff and we would not care about it except when configuring a robot for the first time.