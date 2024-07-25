---
title: Don't Use NetworkManager
---

## NetworkManager and networkd

For historical reasons there are two different network management subsistems on Ubuntu. If you try to use both at the same time, bad things happen. 

## Don't Use NetworkManager

Because of that we don't use NetworkManager and nmcli on any of our robots. Don't install it or set it up. Some of our older robots may still be running NetworkManager. In that case leave it alone and let a Lab technician sort it out.
