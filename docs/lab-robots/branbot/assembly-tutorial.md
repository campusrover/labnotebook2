# BranBot Assembly Tutorial

## Introduction

This is a tutorial on how to assemble the BranBot from the [BranBot List of
Parts](https://docs.google.com/spreadsheets/d/1uRBRJOtii2d5AbC0FSu0Z_0Gutc6lYwzHgX9t3E3YAE/edit?usp=sharing).

'MakerLab' and 'Automation Lab' below refer to the organizations located in the
Brandeis Library.

__WARNING__: Always be careful with tools. Safety comes first!

---
## Overview of Instructions

1. [Prepare Acrylic Frames](#prepare-acrylic-frames)
2. [Attach the LIDAR to the LAF](#attach-the-lidar-to-the-laf)
3. [Attach the LIDAR Board to the LAF](#attach-the-lidar-board-to-the-laf)
4. [Attach the Charlie Board to the LAF](#attach-the-charlie-board-to-the-laf)
5. [Attach the Raspberry Pi to the LAF](#attach-the-raspberry-pi-to-the-laf)
6. [Attach the IMU to the LAF](#attach-the-imu-to-the-laf)
7. [Attach the LED Driver to the LAF](#attach-the-led-driver-to-the-laf)
8. [Attach the Motor Brackets to the LAF](#attach-the-motor-brackets-to-the-laf)
9. [Attach the Motor Assemblies to the Brackets](#attach-the-motor-assemblies-to-the-brackets)
10. [Attach the Wheel Adapters to the Motor Assemblies](#attach-the-wheel-adapters-to-the-motor-assemblies)
11. [Attach the Fixed Wheels to the Wheel Adapters](#attach-the-fixed-wheels-to-the-wheel-adapters)
12. [Attach the Caster Wheels to the LAF](#attach-the-caster-wheels-to-the-laf)
13. [Attach the Camera to the Camera Mount](#attach-the-camera-to-the-camera-mount)
14. [Attach the Camera Mount to the LAF](#attach-the-camera-mount-to-the-laf)
15. [Attach the Battery Brackets to the LAF](#attach-the-battery-brackets-to-the-laf)
16. [Attach the LED Holders to the LAF](#attach-the-led-holders-to-the-laf)
17. [Wiring](#wiring)
    1. [Connect the Raspberry Pi and the Teensy](#connect-the-raspberry-pi-and-the-teensy)
    2. [Connect the LED Board with the Raspberry Pi](#connect-the-led-board-with-the-raspberry-pi)
    3. [Connect the Raspberry Pi and the LIDAR Board to the Battery](#connect-the-raspberry-pi-and-the-lidar-board-to-the-battery)
    4. [Connect the Battery to the Charlie Board Screw Terminal](#connect-the-battery-to-the-charlie-board-screw-terminal)
    5. [Connect the LED Driver to the IMU](#connect-the-led-driver-to-the-imu) 
    6. [Connect the IMU to the Charlie Board](#connect-the-imu-to-the-charlie-board)
    7. [Connect the Motor Assembly Wires to the Charlie Board](#connect-the-motor-assembly-wires-to-the-charlie-board)
    8. [Connect the LEDs to the LED Driver](#connect-the-leds-to-the-led-driver)
    9. [Connect the Camera to the Raspberry Pi](#connect-the-camera-to-the-raspberry-pi)
18. [Attach the Standoffs to the LAF](#attach-the-standoffs-to-the-laf)
19. [Attach the Upper Acrylic Frame to the Standoffs](#attach-the-upper-acrylic-frame-to-the-standoffs)
---

## Prepare Acrylic Frames

1. Parts: Lower Acrylic Frame, Upper Acrylic Frame
2. Tools: Curved Nose Plier 
3. Steps:
    1. It's best to do this step in the Automation Lab, where a curved nose
       plier is readily available.
    2. Take a curved nose plier and punch through any incompletely drilled holes
       in the Lower and Upper Acrylic Frames.

## Attach the LIDAR to the LAF 

1. Note: 'LAF' stands for 'Lower Acrylic Frame'
2. Parts: LIDAR, 4 M2.5x6 screws
3. Tools: Screwdriver
4. Steps:
    1. Align the LIDAR to the correct holes on the LAF.
    2. Attach the LIDAR to the LAF with the 4 M2.5x6 bolts

![Branbot](/labnotebook2/images/branbot/lidar.png){: .center}

## Attach the LIDAR Board to the LAF

1. Parts: LIDAR Board, 4 nylon M2 screws, 4 nylon M2 standoffs, 4 nylon M2 nuts
2. Tools: Screwdriver
3. Steps:
    1. Fix the 4 nylon standoffs to the appropriate holes in the LAF with the 4
       nylon M2 screws.
    2. Insert the LIDAR Board onto the nylon standoffs via its 4 holes.
    3. Fix the LIDAR Board onto the nylon standoffs via the 4 nylon M2 nuts.

![Branbot](/labnotebook2/images/branbot/lidar-board.png){:.center}


## Attach the Charlie Board to the LAF

1. Parts: Charlie Board, 4 nylon M2 screws, 4 nylon M2 standoffs, 4 nylon M2
   nuts
2. Tools: Screwdriver
3. Steps:
    1. Observe that the Charlie Board should be attached to the underside of the
       LAF as in the picture below.
    2. Attach the Charlie Board to the LAF using the nylon screws, standoffs,
       and nuts (cf. the LIDAR board to LAF attachment instructions).
4. Notes:
    1. The Charlie Board should already have the Motor Driver, the Teensy, the
       Screw terminal, and the I2C right-angle connector soldered onto it. The
       terminal and the connector should have been directly soldered on. On the
       other hand, the Motor Driver and the Teensy should have male pins
       soldered onto them, which should have been inserted into female plugs
       that have themselves been soldered to the Charlie Board.
    2. If the above is not the case, ask the Automation Lab for assistance.

![Branbot](/labnotebook2/images/branbot/charlie-board.png){:.center}

## Attach the Raspberry Pi to the LAF

1. Parts: Raspberry Pi, 4 nylon M2 screws, 4 nylon M2 standoffs, 4 nylon M2 nuts
2. Tools: Screwdriver
3. Steps:
    1. Find the appropriate holes in the LAF for the Raspberry Pi.
    2. Attach the Pi to the LAF using the nylon screws, standoffs, and nuts (cf.
       the LIDAR board to LAF attachment instructions).

![Branbot](/labnotebook2/images/branbot/raspberry-pi.png){:.center}

## Attach the IMU to the LAF

1. Parts: IMU, 4 nylon M2 screws, 4 nylon M2 standoffs, 4 nylon M2 nuts
2. Tools: Screwdriver
3. Steps:
    1. Find the appropriate holes in the LAF for the IMU.
    2. Attach the IMU to the LAF using the nylon screws, standoffs, and nuts (cf.
       the LIDAR board to LAF attachment instructions).

![Branbot](/labnotebook2/images/branbot/imu.png){:.center}

## Attach the LED Driver to the LAF 

1. Parts: LED Driver, 4 nylon M2 screws, 4 nylon M2 standoffs, 4 nylon M2 nuts
2. Tools: Screwdriver
3. Steps:
    1. Find the appropriate holes in the LAF for the LED Driver.
    2. Attach the LED Driver to the LAF using the nylon screws, standoffs, and
       nuts (cf.  the LIDAR board to LAF attachment instructions).
4. Notes:
    1.  The LED Driver is the wrong side up in the picture below. So are the
        male pins that have been attached to it. This does not affect the
        functionality of the Driver, but attaching it the right side up is
        preferable.
    2. The LED driver should already have male pins soldered onto it. If it
       doesn't, ask the Automation Lab for assistance.

![Branbot](/labnotebook2/images/branbot/led-driver.png){:.center}

## Attach the Motor Brackets to the LAF

1. Parts: 2 Motor Brackets, 8 M3x10 bolts, 8 M3 nuts
2. Tools: Screwdriver, Plier
3. Steps:
    1. Align a bracket's four holes that form a square with appropriate holes
       in the LAF. 
    2. Fix the bracket to the Frame with bolts and nuts. It helps to hold a nut
       in place with a plier while screwing a bolt into it.
    3. Repeat for the other side.

![Branbot](/labnotebook2/images/branbot/motor-bracket.png){:.center}

## Attach the Motor Assemblies to the Brackets

1. Parts: Motor Brackets, 2 Motor Assemblies, 12 M3x6 bolts
2. Tools: Screwdriver
3. Steps:
    1. Ensure that the shaft of a Motor Assembly is positioned such that it
       occupies the lower portion of the large hole in a Motor Bracket. 
    2. Align the holes in the Motor Assembly with those in the Motor Bracket.
    3. Fix the Motor Assembly with the Motor Bracket using 6 M3x6 bolts.
    4. Repeat for the other side.
4. Note: The length of the bolts should be no longer than 6mm. Otherwise, they
   might interfere with the functioning of the gear box in the Motor Assembly.

![Branbot](/labnotebook2/images/branbot/motor-assembly.png){:.center}

## Attach the Wheel Adapters to the Motor Assemblies

1. Parts: 2 Wheel Adapters (thick parts, see picture below), 4 Stud bolts (these
   come with the Pololu wheel adapters, and are small headless bolts) 
2. Tools: Screwdriver
3. Steps:
    1. Find the flat edge of the shaft of a Motor Assembly.
    2. Insert a wheel adapter onto the shaft and rotate it such that the two
       holes on the adapter's side align with the mentioned flat edge. Ensure
       that:
       1. the shaft does not stick out through the adapter, but that the end of
          the shaft is flush with the adapter (cf. the picture below). 
       2. The thinner end of the thick wheel adapter faces outward (cf. the
          picture below).
    3. Insert two stud bolts into the two holes to fix the adapter to the shaft.
    4. Repeat for the other side.

![Branbot](/labnotebook2/images/branbot/wheel-adapter-thick.png){:.center}

## Attach the Fixed Wheels to the Wheel Adapters

1. Parts: 2 Wheel Adapters (thin parts, see picture below), 6 M3x20 bolts
2. Tools: Screwdriver
3. Steps:
    1. Insert a Fixed Wheel into the part of the Wheel Adapter that is attached
       to the Motor Assembly.
    2. Insert the thin part of the Wheel Adapter into the central hole of the
       wheel, ensuring that its three holes align with that in the thick part of
       the wheel adpater.
    3. Fix the thin part of the Wheel Adapter to its thick counterpart by
       inserting three M3x20 bolts into the three holes.
    4. Repeat for the other side.
4. Note: The length of the bolts should be no longer than 20mm. Otherwise, they
   might interfere with the rotation of the Motor Assembly's shaft. 

![Branbot](/labnotebook2/images/branbot/wheel-adapter-thin.png){:.center}
![Branbot](/labnotebook2/images/branbot/fixed-wheel.png){:.center}

## Attach the Caster Wheels to the LAF

1. Parts: Caster Wheel Offsets, Caster Wheels, 8 M6x20 bolts, 8 M6 nuts
2. Tools: Screwdriver, Pliers
3. Steps:
    1. Use the pliers to remove the breaks from a Caster Wheel.
    2. Align the holes of a Caster Wheel Offset and those of a Caster
       Wheel to an appropriate set of holes in the LAF.
    3. Use hands to loosely fix the Caster Wheel Offset, the Caster Wheel, and
       the LAF via the M6 nuts and bolts.
    4. Use the plier to hold the nuts in place and the screwdriver to fasten the
       bolts into the nuts.
    5. Repeat for the other side.
4. Notes:
    1. The Caster Wheel Offsets are manufactured by MakerLab. 

![Branbot](/labnotebook2/images/branbot/caster-wheels.png){:.center}

## Attach the Camera to the Camera Mount

1. Parts: 2 M2x4 screws, Camera, Camera Mount
2. Tools: Screwdriver
3. Steps:
    1. Fasten the Camera to the Camera Mount with the screws.
4. Note: Camera Mounts are printed by the MakerLab.

![Branbot](/labnotebook2/images/branbot/camera-to-mount.png){:.center}

## Attach the Camera Mount to the LAF

1. Parts: 2 M3x16 bolts, 2 M3 nylon lock nuts, Camera Mount
2. Tools: Pliers Screwdriver
3. Steps:
    1. Fasten the Camera Mount to the LAF using the bolts and nylon nuts. Hold
       the nylon nut in place with a plier when fastening the bolt. 

![Branbot](/labnotebook2/images/branbot/mount-to-laf.png){:.center}

## Attach the Battery Brackets to the LAF

1. Parts: 2 Battery Brackets with heat-set nuts, 2 foam inserts, 4 M3x10 bolts
2. Tools: Screwdriver
3. Steps:
    1. Observe the orientation of the battery relative to the brackets in the
       pictures below.
    2. Insert the battery into the brackets, using the foam inserts to secure it
       in place. 
    3. Attach the battery to the LAF using the 4 M3x10 bolts
4. Notes:
    1. The battery brackets should already have heat-set nuts set into them. If
       this isn't the case, ask for assistance from the MakerLab.
    2. The MakerLab is also responsible for providing the foam inserts.

![Branbot](/labnotebook2/images/branbot/bracket-one.png){:.center}
![Branbot](/labnotebook2/images/branbot/bracket-two.png){:.center}
![Branbot](/labnotebook2/images/branbot/bracket-to-laf.png){:.center}

## Attach the LED Holders to the LAF

1. Parts: 4 M3x10 screws, 4 LED Holders
2. Tools: Screwdriver
3. Steps:
    1. Insert the LED bulbs/wires into the LED holders.
    2. Fix the LED holders to the appropriate holes in the LAF with the screws.
4. Notes:
    1. The LED Holders should have heat-set nuts melted into them. If they do
       not, ask for assistance from the Automation Lab.
    2. The LED wires should have bulbs on one end and female plugs on the other.
       If the female plugs are missing, ask for assistance from the Automation
       Lab.

![Branbot](/labnotebook2/images/branbot/led-front.png){:.center}
![Branbot](/labnotebook2/images/branbot/led-back.png){:.center}

## Wiring

### Connect the Raspberry Pi and the Teensy 

1. Part: USB 2.0 to Micro USB Right-Angle Connector.

![Branbot](/labnotebook2/images/branbot/pi-to-teensy-above.png){:.center}
![Branbot](/labnotebook2/images/branbot/pi-to-teensy-below.png){:.center}

### Connect the LED Board with the Raspberry Pi 

1. Part: USB 2.0 to Micro USB Cable.

![Branbot](/labnotebook2/images/branbot/led-board-to-pi.png){:.center}

### Connect the Raspberry Pi and the LIDAR Board to the Battery

1. Part: USB 2.0 to Micro USB and USB-C Splitter Charging Cable 

![Branbot](/labnotebook2/images/branbot/pi-lidar-to-battery-above.png){:.center}
![Branbot](/labnotebook2/images/branbot/pi-lidar-to-battery-battery.png){:.center}

### Connect the Battery to the Charlie Board Screw Terminal 

1. Part: Battery Power Cable

![Branbot](/labnotebook2/images/branbot/cb-to-battery-below.png){:.center}
![Branbot](/labnotebook2/images/branbot/cb-to-battery-battery.png){:.center}

### Connect the LED Driver to the IMU

1. Part: I2C Cable
2. Note: The LED Driver is the wrong side up in the picture below. So are the
   male pins that have been attached to it. This does not affect the
   functionality of the Driver, but attaching it the right side up is
   preferable.

![Branbot](/labnotebook2/images/branbot/led-driver-to-imu.png){:.center}

### Connect the IMU to the Charlie Board

1. Part: I2C Cable

![Branbot](/labnotebook2/images/branbot/imu-to-cb-above.png){:.center}
![Branbot](/labnotebook2/images/branbot/imu-to-cb-below.png){:.center}

### Connect the Motor Assembly Wires to the Charlie Board

![Branbot](/labnotebook2/images/branbot/motor-to-cb.png){:.center}

### Connect the LEDs to the LED Driver

![Branbot](/labnotebook2/images/branbot/led-to-driver-side.png){:.center}
![Branbot](/labnotebook2/images/branbot/led-to-driver-above.png){:.center}

### Connect the Camera to the Raspberry Pi

1. Part: Raspberry Pi Camera Ribbon Cable
2. Note: Observe how the ribbon cable twists as it goes from the pi to the
   camera.

![Branbot](/labnotebook2/images/branbot/pi-to-cam-side.png){:.center}
![Branbot](/labnotebook2/images/branbot/pi-to-cam-front.png){:.center}

## Attach the Standoffs to the LAF

1. Parts: 6 75mm standoffs, 6 M3x10 screws
2. Tools: Screwdriver
3. Steps:
    1. Attach the standoffs to the appropriate holes on the LAF with the screws.

![Branbot](/labnotebook2/images/branbot/standoffs-to-laf-one.png){:.center}
![Branbot](/labnotebook2/images/branbot/standoffs-to-laf-two.png){:.center}

## Attach the Upper Acrylic Frame to the Standoffs

1. Parts: 6 M3x10 screws
2. Tools: Screwdriver
3. Steps:
    1. Attach the Upper Acrylic Frame to the Standoffs with the M3x10 screws.

![Branbot](/labnotebook2/images/branbot/uaf-to-standoffs-one.png){:.center}
![Branbot](/labnotebook2/images/branbot/uaf-to-standoffs-two.png){:.center}

