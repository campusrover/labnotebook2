---
title: Don't Use NetworkManager
---

## NetworkManager and networkd

For historical reasons there are two different network management subsistems on Ubuntu. If you try to use both at the same time, bad things happen. 

## Don't Use NetworkManager

Because of that we don't use NetworkManager and nmcli on any of our robots. Don't install it or set it up. Some of our older robots may still be running NetworkManager. In that case leave it alone and let a Lab technician sort it out.


## How to Uninstall NetworkManager

For instructions on how to uninstall NetworkManager, please refer to our [NetworkManager Uninstallation Guide](../uninstall-networkmanager.md). This guide provides step-by-step instructions for safely removing NetworkManager from your system without disrupting your network connectivity.


