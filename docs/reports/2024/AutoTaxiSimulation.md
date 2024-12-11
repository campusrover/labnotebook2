---
title: Auto Taxi Simulation
author: Chao An
date: Dec 10 2024
---
## Introduction
As autonomous driving is being adopted by all major EV companies, it is interesting for me to imagine what the autonomous driving system will be like in the future. And I would like to imitate it in ROS.

For the initial goal, 2 robots should be able to drive around in a known road system, and the robot that is closest to the passengers would go and pick them up. They will follow basic traffic rules like stopping for stop signs and signal lights, which are represented as Fiducials. They should also be able to handle scenarios like unexpected pedestrians crossing the street and clearing the way for emergency cars. However, later I found there was too much work for a solo project to work on 2 robots, so I decided to complete 1 robot following the traffic system.

## What was created
### Technical descriptions, illustrations

The robot operates in a mapped space containing several fiducials. These fiducials represent either signal lights or stop signs.
For mapping the area, I use the gmapping method based on SLAM, implemented from an existing library. The robot navigates to its location using the move_base library.
For fiducial recognition, I utilize the aruco_detect library. The fiducials' locations are stored in the robot's odometry system after being scanned by the robot's camera.
For detecting unexpected objects or pedestrians on the street, the robot uses LIDAR to identify obstacles and plans new paths to avoid them.

### Discussion of interesting algorithms, modules, techniques

1. Basic Mapping and Navigation
For this project, I integrated three key components to achieve autonomous navigation: gmapping for environment mapping, move_base for path planning, and Lidar sensing for real-time perception. The gmapping package implements SLAM techniques to construct a global map as the robot explores it prior to navigation. Using LIDAR sensor data, the system captures any components that weren't scanned during the mapping phase and uses it to create a local map to avoid unmapped obstacles.
With the environment mapped, the move_base package takes charge of navigation tasks. It processes the map data to generate a cost map, which helps the robot understand where it can safely travel. Before initiating any forward movement, the robot first rotates to align itself with its planned path direction, ensuring smooth and efficient navigation. This navigation system not only plans efficient routes but also ensures compliance with traffic rules detected through fiducial markers. The robot's LIDAR sensor continuously monitors its surroundings, enabling real-time detection and avoidance of dynamic obstacles that weren't present during the initial mapping phase, such as crossing pedestrians.

2. Dealing with Signals & Stop Signs
The traffic control system uses fiducial markers to represent signal lights and stop signs. When the robot approaches a signal light fiducial, it subscribes to a ROS topic that updates the light status every 30 seconds. Upon detecting a red light, the system stores the current move_base goal and temporarily halts navigation. Navigation automatically resumes when the signal turns green.
For stop sign management, the system maintains an array of vehicles waiting at the intersection. When a robot encounters a stop sign, it adds itself to this queue and comes to a complete stop. After a mandatory 3-second wait, the first robot in the queue is permitted to proceed, and its entry is removed from the array. Subsequent robots must remain stopped until they reach the front of the queue, implementing a first-come-first-serve traffic management system.

3. Pickup passengers
For passenger management, the system maintains a queue of pickup and dropoff locations, each stored as XYZ coordinates. The robot processes these locations sequentially, navigating to each pickup/drop off point in order. Once a passenger location is reached, that destination is removed from the queue, and the robot proceeds to the next location. This simple yet effective system ensures orderly passenger service on a first-come-first-served basis.

### Guide on how to use the code written

To run the program, use command
```
roslaunch ros_auto_taxi ros_auto_taxi.launch
```

To run the program by itself with demo passenger locations, use this command after run
```
rosrun ros_auto_taxi demo_day.py
```

### Clear description and tables of source files, nodes, messages, actions and so on

| File name | Description |
|----------|----------|
| four_way_solo.py | include reaction towards signal/stop sign |
| mapper_real.py | mapping fiducial location |
| my_odom_solo.py | retrieve useful odom information |
| signal_sim.py | publish signal lights shift |

| Message Name | Description | Message Type |
|-------------|-------------|--------------|
| /scan | LIDAR scan data for obstacle detection and mapping | sensor_msgs/LaserScan |
| /move_base/status | Navigation status updates from move_base | actionlib_msgs/GoalStatusArray |
| /move_base/goal | Target pose for navigation | move_base_msgs/MoveBaseActionGoal |
| /move_base/cancel | Cancel current navigation goal | actionlib_msgs/GoalID |
| /cmd_vel | Robot velocity commands | geometry_msgs/Twist |
| /signal_sim | Traffic light status updates | std_msgs/Bool |
| /stop_sign_sim | Stop sign detection and management | std_msgs/Int32 |
| /odom | Robot odometry data (position/velocity) | nav_msgs/Odometry |
| /my_odom | A simplified odom topic | std_msgs/Float32MultiArray |

## Story of the project. 
1. How it unfolded, how the team worked together

### Your own assessment
The project successfully implements basic autonomous taxi functionality, demonstrating effective integration of mapping, navigation, and traffic rule compliance. The system achieves its core objectives of autonomous movement, traffic sign response, and passenger management.

There are two main areas for potential improvement: implementing Computer Vision for more realistic traffic sign detection beyond fiducial markers, and expanding to a multi-robot system as originally envisioned. These enhancements would bring the system closer to real-world autonomous taxi operations.
