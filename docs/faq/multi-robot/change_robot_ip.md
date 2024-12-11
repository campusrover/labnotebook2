---
title: How to change robot ip
author: Haochen Lin
status: new
date: Dec 2024
---

The Project runs multiple robots at the same time. By signing one robot to another robot’s IP, we can control two robots at the same time. However, due to environmental limitations, we can not run more than two robots since we don’t want to interrupt others by corrupting the robot when we sign it to another robot.

Step1: 

make sure robot 1 is on board through ssh

then type

```bash

nano ~/.bashrc

```

Step2: 

change the IP address to robot2

Then

```bash

source ~/.bashrc

```

save bashrc changes


Then you should be able to launch the robot1 using

```bash

roslaunch turtlebot3_bringup turtlebot3_multi_robot.launch

```

