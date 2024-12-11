---
title: gmapping Parameters Recommendation
author: Chao An
date: Dec 7 2024
---
## Author
* Chao An
* Dec 7 2024
* ROS 1

## Summary

Recommendations for gmapping parameters and location of it.

## Details

gmapping is the most beginner friendly SLAM algorithm that is provided in ROS1. The limitation of gmapping is that it must used with robots that provide /scan rostopic (Lidar) and /odom rostopic (Odom). There's other options like Karto SLAM and Hector SLAM, but gmapping would be the one I'm choosing for my project.

But gmapping with default parameters might not be appropriate for all environment like maze. Here's my recommend parameters for small environment. 

## Instructions
To change the parameters, you can modify: 
```bash
turtlebot3/turtlebot3_slam/config/gmapping_params.yaml .
```
It would be called by command like: 
```bash
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```

## Parameters
Here's the parameters: 
```bash
map_update_interval: 2.0
maxUrange: 3.0
sigma: 0.05
kernelSize: 1
lstep: 0.05
astep: 0.05
iterations: 5
lsigma: 0.075
ogain: 3.0
lskip: 10 # Somebody on forus use 0 or 10
minimumScore: 10 # Options: 10, 50(default)
srr: 0.01 # 0.01 , 0.1 (default)
srt: 0.02 # 0.02, 0.2 (default)
str: 0.01 # 0.01, 0.1 (default)
stt: 0.02 # 0.02, 0.2 (default)
linearUpdate: 0.05 # 0.05, 0.1 (default)
angularUpdate: 0.2 # 0.2, 0.1 (default)
temporalUpdate: 0.5
resampleThreshold: 0.5
particles: 100
xmin: -10.0
ymin: -10.0
xmax: 10.0
ymax: 10.0
delta: 0.01 # 0.01, 0.05 (default)
llsamplerange: 0.01
llsamplestep: 0.01
lasamplerange: 0.005
lasamplestep: 0.005
```

## Clarification
Hopefully these parameters recommendation can help you, but it is important to modify it based on your using condition. Remember, tunning mapping is closer to art than science, which means you need luck sometimes.