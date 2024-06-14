---
title: Teensy and Udev
author: Pito Salas
description: Different model teensy
status: new
created: 14-jun-2024
version: 1
---
# Teensy Properties

Udev rules recognize the Teensy by a few properties that they can detect when the teensy is plugged in.

## Product ID or PID for Teensy
* Teensy 1.0 uses PID 0x0487
* Teensy++ 1.0 uses PID 0x04AE
* Teensy 2.0 uses PID 0x04B2
* Teensy 3.0/3.1/3.2 use PID 0x04B3
* Teensy 3.5 uses PID 0x04B5
* Teensy 3.6 uses PID 0x04B6
* Teensy 4.0 uses PID 0x04B7

## Vendor ID or VID for Teensy
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

## Udevadm - Udev admin

This has many commands. The ones I have used are:

`Udevadm monitor`

`udevadm monitor --property`

## Python Serial Tools 

`python -m pip install pyserial`

With that tool, now you can list the information about each device plugged into a USB with this little app:

```
from serial.tools import list_ports
ports = list_ports.comports()
for port in ports:
    print(f"Device: {port.device}, Name: {port.name}, Description: {port.description}, HWID: {port.hwid}")
```

## Linrobot's lino_udev package

This packate is problematic because it requires Python2