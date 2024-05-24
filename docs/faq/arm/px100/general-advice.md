---
title: Any general advice before I work with the PX-100?
desc: General Advice on the PX-100 
date: may-2024 
status: new
author: James Lee 
type: faq
---

# Question
Any general advice before I start working with the PX-100?

# Answer

1. _Fix the bug in Interbotix's source code._ There's a bug in Interbotix's
   source code that you should fix before you start working with the arm.
Unless, that is, someone already fixed it for you. Ask Professor Salas about
this, and read the entry `api-fix.md`.
2. _Study previous projects that used the arm._ The most helpful are probably
   [Fiducial Pick and
Place](https://github.com/campusrover/fiducial_pick_and_place) and
[Pnp](https://github.com/campusrover/pnp). But this is only because they in turn
referenced previous projects, like
[ArmCam](https://github.com/campusrover/ArmCam/).
3. _When stuck and previous projects don't provide an answer, dig into
   Interbotix's source code._ Most of the source code you'll need is probably
[here](https://github.com/Interbotix/interbotix_ros_toolboxes). Yes, it's a lot
of code, and to make sense of it you might have to dig into other repositories
that interbotix offers. But this is good practice for working with code "in the
real world". You'll have to do it anyway, presuming you want to work in
industry. Why not start now?
4. _Sometimes your computer just fails to detect the arm. RViz will show scary
   white boxes where the robot's limbs should be._ Don't worry! You (probably)
didn't break the arm. Just do a hard restart of it: unplug and replug the data
and power chords from the arm. That should fix the problem.
5. _Getting the arm to move as you want is hard. Have patience, and try to
   troubleshoot in a methodical, rational manner._ It's really hard! Especially
if you don't have a strong math background, and have to rely on Interbotix's
official API. The `set_ee_cartesian_trajectory` method, for example, tells you
how much of the trajectory the API successfully calculated, but not why it
failed or succeeded as much as it did. Still, don't worry: there are probably
easier ways around your problem than trying to do a crash course on inverse or
forward kinematics. Reference previous projects done at the lab and see how they
solved their problems.

