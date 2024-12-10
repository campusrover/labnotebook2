---
title: Using move_base in gazebo
author: Daphne Pissios
date: Dec 9 2024
---
move_base is a ROS navigation package that allows a robot to navigate through an environment. It provides path planning and obstacle avoidance in the navigation. The move_base node creates a server that accepts navigation goals and attempts to guide the robot from its current position to the specified target position while avoiding obstacles. Using move_base, you can also create a map of the environment. To send a goal to move_base, you can use RViz or a script, specifying a target pose in the environment.

To launch move_base you can use:
Open the gazebo world you want to navigate
```roslaunch turtlebot3_gazebo turtlebot3_<gazebo_world>.launch```

Start the mapping algorithm with 
```roslaunch turtlebot3_slam turtlebot3_gmapping.launch```

Start RViz  
```roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch```
Make sure to add the map display in RViz

To create a goal using RViz, use the 2D Nav Goal button at the top of the screen, you can also use another script or use teleop to move the robot around. When the robot moves around, the map will be visibly filled out. Impassible walls will appear as black lines.

To save a created map run:
```rosrun map_server map_saver -f <filename>```

To open up gazebo with a created map run
```roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:="<filepath.yaml>"```

The yaml file created when saving a map will look something like this:
``` 
    image: test_map.pgm
    resolution: 0.050000
    origin: [-10.000000, -10.000000, 0.000000] 
    negate: 0
    occupied_thresh: 0.65
    free_thresh: 0.196
```
Changing the origin will change where the robot centers the map. The occupied and free thresholds determine whether a cell on the map will be considered impassible or not.
