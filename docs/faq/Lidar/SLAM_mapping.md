---
title: Mapping surrounding with SLAM
author: TsunOn Kwok
date: Dec 10 2024
---
## Author
* TsunOn Kwok
* Dec 10 2024
* Mapping surrounding with SLAM

## Summary

Brief instructions for how ot mapping a surrounding with SLAM in both real robot and gazebo

## Details

To map the surrouding and generate a `.yaml` world file and a `.pgm` map, follow these steps:

1. Bring up the robot in real life or launch you robot and world in gazebo to get started.

2. Launch SLAM using `gmapping` by running the following command:

   ```bash
   roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
   ```
    Rviz should be automatically opened after the command is typed.

3. Open a new terminal and run the command: `teleop` to control the robot with the keyboard. Drive the robot around the target area to scan its surroundings.

4. Once the entire target area has been scanned, open another terminal and execute the following command to save the map:

   ```bash
   rosrun map_server map_saver -f `rospack find <package_name>`/maps/<map_name>
   ```

5. This will generate two files in your Ros package `<package_name>/maps`: `<map_name>.yaml` and `<map_name>.pgm`.

### Things to notice
---
- In gazebo, due to lagging and its poor accuary, the SLAM mapping is very unaccuary. For any world over 10x10 meters, the SLAM mapping is unusable. It is highly remmcomended that you keep the world as small as poposible in gazebo. Our gazebo world is 5x5 meters.


- Adjust SLAM Parameters: Modify parameters like linearUpdate, angularUpdate, and particles in the gmapping configuration file to suit your robot and environment.