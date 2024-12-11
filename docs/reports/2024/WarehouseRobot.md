# Warehouse Robot Project

**Author:** Eric Hurchey  
**Date:** Dec 10, 2024  

## Introduction

### Problem Statement
The project aimed to design and implement a warehouse robot capable of autonomously following a predefined line, locating a specific fiducial marker, turning around upon reaching the marker, and returning to its initial position. The robot also needed to handle obstacles and maintain accurate navigation.

### Relevant Literature
- Concepts of PID control for navigation.
- Vision-based line detection using HSV color space.
- Techniques for fiducial marker localization and pose estimation.
- Autonomous robot navigation principles using ROS (Robot Operating System).

### Team Members
- Eric Hurchey ([erichurchey@brandeis.edu](mailto:erichurchey@brandeis.edu))

## What Was Created

### Technical Descriptions
- The robot was built using ROS for communication, control, and sensor data processing.
- A Raspberry Pi camera provided input for line following through real-time image processing.
- The robot utilized Odometry data and TF transformations to localize and navigate towards fiducial markers.

### Interesting Algorithms, Modules, Techniques
- **PID Controller for Line Following:** A PID controller adjusted the robot’s angular velocity based on the error calculated from the detected line’s center and the image’s center.
- **Fiducial Marker Detection:** Used ROS TF to transform fiducial marker coordinates into the robot’s base frame to calculate distance and angle for navigation.
- **State Machine:** Managed the robot’s tasks, transitioning between states: `LINE_FOLLOWING`, `APPROACHING`, `TURNING_AROUND`, and `RETURNING`.

### Guide on How to Use the Code
1. Run the needed bring up files:
  roslaunch Warehouse-Robot robot_bringup.launch 
2. Run the node by executing the Python script:
   rosrun warehouse_robot warehouse_robot.py
