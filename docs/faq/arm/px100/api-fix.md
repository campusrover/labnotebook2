---
title: How do I fix the official Interbotix API? 
desc: How to fix the official Interbotix API 
date: may-2024 
status: new
author: James Lee 
type: faq
---

# Question
I heard that Interbotix's official API has a bug. How do I fix it? 

## Problem 

The arm’s official API is unable to process rapid sequential movement
commands. That is, suppose `move(p)` represents an API command to move the arm’s
gripper to some point `p` within a given coordinate frame. Roughly speaking,
then, the arm is unable to comply when we execute `move(p1)`, `move(p_2)`, and
`move(p_3)` in rapid succession.

## Solution

Modify the `queue_size` arguments of three ROS publishers in the arm's API
source code.

### Steps

1. Download Interbotix's official API by following the instructions
   [here](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros1/software_setup.html).

2. Open `interbotix_ws/src/interbotix_ros_toolboxes/
interbotix_xs_toolbox/interbotix_xs_modules/src/interbotix_xs_modules/core.
py`

3. Change the `queue_size` argument of the `pub_group`, `pub_single`, and
   `pub_traj` publishers from `1` to `20`.

### Why does this work?

__If you don't have time, skip this section.__

To see why the fix works, we should first get a brief overview of how the arm’s
movement API behaves under the hood. 

A movement function of the arm API works by publishing command messages over one
or more of three topics:

1. `JointSingleCommand` messages over `/px100/commands/joint_single`,
2. `JointGroupCommand` messages over `/px100/commands/joint_group`, or
3. `JointTrajectoryCommand` messages over `/px100/commands/joint_trajectory`.  

And when a message is published over one of these three topics, the
`/px100/xs_sdk` node that subscribes to the topics processes the message and
orders the arm to move as the the message directs.

So if a user calls an API movement function at a rate `R`, the API function in
turn should publish command messages at a rate `R`, which the `/px100/xs_sdk`
node should process at the same rate.

But for any ROS publisher `p`, `p` works by storing the messages it is to
publish in a queue, and processing them sequentially. Further, where `m` is a
message, if we call `p.publish(m)` when `p`’s queue is full, ROS silently drops
`m` and never publishes it. 

Now the `queue_size` of the publishers that the API’s movement functions used
were all set to `1`. Thus, if the rate `R` at which we called an API movement
function was too high, the publishers would often try to publish a message `m`
when its queue was full, thus dropping `m`.  Thus, this problem is solved by
simply increasing the `queue_size` of the mentioned publishers from `1` to a
generous `20`.
