---
title: Teensy and Udev
author: Pito Salas
description: Setting up UDEV for the teensy
status: new
created: 14-jun-2024
version: 1
---
## Intro

As mentioned, the udev subsystem in Ubuntu allows us to recognize a particular USB device no matter what port it is attached to. It is controled by files called udev rules which are all stored in /etc/udev/rules.d. They are all "processed" by the udev subsytem.

## Checking configuration

On the rasberry Pi, type the command `ls /dev/lino*`. It should list two devices, `linolidar` (the lidar) and `linobase` (the teensy). It they appear, you can ignore the rest of this document.

## Fixing the udev configuration

For all our robots based on LinoRobot, we use Teensy controller and they have some particular udev requirements. 

1. Use the official [latest teensy udev rule](https://www.pjrc.com/teensy/00-teensy.rules) from the vendor.
2. Create a udev rule to map the teensy to the standard name, /dev/linobase. The standard name is then in turn used by the code to open the connection to the teensy, 

Copy both these files to the /etc/udev/rules.d directory.

## Creating the linobase udev rule

!!! Important "Don't use lino_udev.py!"
    The Linorobot1 instructions have a little script called lino_udev.py. 
    We **do not use it** because it uses old apis that no longer work!

The basic udev rule file contains lines something like this:
```
KERNEL=="ttyUSB?", SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0001", MODE="0666" SYMLINK+="linolidar"
SUBSYSTEM=="usb", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="0483", ATTRS{serial}=="15412350", MODE="0666" SYMLINK+="linobase"
```
The order does not matter. The first line detects the Lidar USB and links it to the name `linolidar`. The second line is suppsed to detect the teensy and link it to the name `linobase`. However depending on the model teensy we have (and they differ on our robots) the particulars will varry. The key particulars are idVendor, idProduct, and serial.

To get that information we run a little script:

## Script to print out the detected devices

First: `python -m pip install pyserial`

With that tool, now you can list the information about each device plugged into a USB with this little app:

```
from serial.tools import list_ports
ports = list_ports.comports()
for port in ports:
    print(f"Device: {port.device}, Name: {port.name}, Description: {port.description}, HWID: {port.hwid}")
```

In the output to this script you will find the required magic numbers.

## Updating the udev rules

Once you edit the udev rule file, unplug the usb for the teensy from the Pi and plug it back in. The teensy should be detected.


## Handy tool: Udevadm - Udev admin

This has many commands. The ones I have used are:

`Udevadm monitor`

`udevadm monitor --property`

## Linrobot's lino_udev package

This package is problematic because it requires Python2. We will not be using it.

## Teensy Properties

Udev rules recognize the Teensy by a few properties that they can detect when the teensy is plugged in.

## Teensy PID and VID

### Product ID or PID for Teensy
* Teensy 1.0 uses PID 0x0487
* Teensy++ 1.0 uses PID 0x04AE
* Teensy 2.0 uses PID 0x04B2
* Teensy 3.0/3.1/3.2 use PID 0x04B3
* Teensy 3.5 uses PID 0x04B5
* Teensy 3.6 uses PID 0x04B6
* Teensy 4.0 uses PID 0x04B7

### Vendor ID or VID for Teensy
1. Teensy 1.0: Product ID 0483
2. Teensy 2.0: Product ID 0483
3. Teensy++ 1.0: Product ID 0483
4. Teensy++ 2.0: Product ID 0483
5. Teensy 3.0: Product ID 16C0
6. Teensy 3.1: Product ID 16C0
7. Teensy 3.2: Product ID 0483
8. Teensy LC: Product ID 16C0
9. Teensy 3.5: Product ID 16C0
10. Teensy 3.6: Product ID 16C0
11. Teensy 4.0: Product ID 16C0
12. Teensy 4.1: Product ID 16C0

