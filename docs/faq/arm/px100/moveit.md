---
title: How do I use MoveIt to Control the PX-100 Arm?
desc: How to control the PX-100 Arm with MoveIt 
date: may-2024 
status: new
author: James Lee 
type: faq
---

# Question

I heard that MoveIt may be a good way to get acquainted with the PX-100 arm. How
might I go about using it?

## Setup 

Check if you have Interbotix's ROS1 software package installed on your computer.
You can check if you do by executing

```bash
rospack list | grep interbotix
```

in your terminal. This should output the names of 20 or so packages that have
`interbotix_` as a prefix. If you see such output, you're good to go!

If your computer doesn't have Interbotix's ROS1 software package, navigate to
[this
page](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros1/software_setup.html#software-installation),
and follow the appropriate instructions to install the required software.

## Launching RViz with MoveIt

Next, execute this command to launch RViz with MoveIt:

```bash
roslaunch interbotix_xsarm_moveit_interface xsarm_moveit_interface.launch robot_model:=px100 dof:=4 use_actual:=true
```

Note the parameters: we are specifying that we want to use Interbotix's MoveIt
interface with the px100 which has 4 degrees of freedom. We are also stating
that we want to use MoveIt with the actual robot arm, rather than in simulation.

Once you execute the command above, go to the `Displays` window of RViz, and
change the `Fixed Frame` from `world` to `px100/base_link`, as pictured below:

<p align="center">
    <kbd>
        <img src="../../../images/moveit1.png />
    </kbd>
</p>





