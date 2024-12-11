---
title: Frontier Exploration Guide
author: ZHENXU Chen
date: Dec 10, 2024
---
## Overview

Frontier exploration is a key capability for mobile robots navigating unknown environments. This guide explains the concept, scenarios, implementation strategies, and practical usage in ROS Noetic.

---

## Author

- **Name:** ZHENXU Chen
- **Date:** Dec 10, 2024
- **ROS Version:** Noetic

---

## Concept

### Key Strategies

Frontier exploration uses map updates to identify unexplored areas and direct robots safely. Integration with the ROS navigation stack ensures obstacle avoidance and adaptability to environmental changes.

### Frontier Detection Algorithm

The detection process involves:

1. **Robot Pose Retrieval:** Using `getRobotPose` to determine the current position.
2. **Frontier Search:** `searchFrom` performs Breadth-First Search (BFS) to locate boundaries between known and unknown areas, sorted by cost.
3. **Empty Frontier Handling:** If no frontiers are found, exploration stops.
4. **Visualization Markers:** Publish visual markers for debugging or monitoring.
5. **Target Selection:** Selects a valid frontier centroid as the next goal.

### Explore Lite

Explore Lite is a ROS package for lightweight autonomous exploration. It provides tools for efficient navigation and mapping in unknown environments.

---

## Usage

### Path Planning Example

```xml
<launch>
  <arg name="model" default="$(env TURTLEBOT3_MODEL)" doc="Model type [burger, waffle, waffle_pi]"/>
  <arg name="cmd_vel_topic" default="/cmd_vel" />
  <arg name="odom_topic" default="odom" />
  <arg name="move_forward_only" default="false"/>

  <node pkg="move_base" type="move_base" name="move_base" output="screen">
    <param name="base_local_planner" value="dwa_local_planner/DWAPlannerROS" />
    <param name="base_global_planner" value="global_planner/GlobalPlanner"/>
    <rosparam file="$(find turtlebot3_navigation)/param/costmap_common_params_$(arg model).yaml" command="load" ns="global_costmap" />
    <rosparam file="$(find turtlebot3_navigation)/param/costmap_common_params_$(arg model).yaml" command="load" ns="local_costmap" />
    <rosparam file="$(find turtlebot3_navigation)/param/local_costmap_params.yaml" command="load" />
    <rosparam file="$(find turtlebot3_navigation)/param/global_costmap_params.yaml" command="load" />
    <rosparam file="$(find turtlebot3_navigation)/param/move_base_params.yaml" command="load" />
    <rosparam file="$(find turtlebot3_navigation)/param/dwa_local_planner_params_$(arg model).yaml" command="load" />
    <remap from="cmd_vel" to="$(arg cmd_vel_topic)"/>
    <remap from="odom" to="$(arg odom_topic)"/>
    <param name="DWAPlannerROS/min_vel_x" value="0.0" if="$(arg move_forward_only)" />
  </node>
</launch>
```

### SLAM Example

```xml
<launch>
  <arg name="model" default="$(env TURTLEBOT3_MODEL)" doc="Model type [burger, waffle, waffle_pi]"/>
  <arg name="slam_methods" default="gmapping" doc="SLAM type [gmapping, cartographer, hector, karto, frontier_exploration]"/>
  <arg name="configuration_basename" default="turtlebot3_lds_2d.lua"/>
  <arg name="open_rviz" default="true"/>

  <include file="$(find turtlebot3_bringup)/launch/turtlebot3_remote.launch">
    <arg name="model" value="$(arg model)" />
  </include>

  <include file="$(find turtlebot3_slam)/launch/turtlebot3_$(arg slam_methods).launch">
    <arg name="model" value="$(arg model)"/>
    <arg name="configuration_basename" value="$(arg configuration_basename)"/>
  </include>

  <group if="$(arg open_rviz)">
    <node pkg="rviz" type="rviz" name="rviz" required="true"
          args="-d $(find turtlebot3_slam)/rviz/turtlebot3_$(arg slam_methods).rviz"/>
  </group>
</launch>
```

### Exploration Configuration Example

```xml
<launch>
  <node pkg="explore_lite" type="explore" respawn="false" name="explore" output="screen">
    <param name="robot_base_frame" value="base_footprint" />
  
    <param name="costmap_topic" value="move_base/global_costmap/costmap" />
    <param name="costmap_updates_topic" value="move_base/global_costmap/costmap_updates" />
    <param name="visualize" value="true" />
    <param name="planner_frequency" value="0.20" />
  
    <param name="progress_timeout" value="30.0" />
    <param name="potential_scale" value="3.0" />
    <param name="orientation_scale" value="0.0" />
    <param name="gain_scale" value="1.0" />
  
    <param name="transform_tolerance" value="0.3" />
    <param name="min_frontier_size" value="0.1" />
  </node>
</launch>
```

### Communication Between Nodes

![Robot Image](frontier-exploration.png)

## Troubleshooting

### Path Planning Failures

Use recovery behaviors and parameters to handle path planning failures during robot navigation. The recovery behaviors include clearing costmaps at varying levels of conservativeness and performing rotation-based costmap clearing. Key parameters such as planner_patience, controller_patience, conservative_reset_dist, planner_frequency, and oscillation_timeout ensure robust and reliable robot navigation by preventing getting stuck.

```yaml
recovery_behaviors:
  - name: 'super_conservative_reset1'
    type: 'clear_costmap_recovery/ClearCostmapRecovery'
  - name: 'conservative_reset1'
    type: 'clear_costmap_recovery/ClearCostmapRecovery'
  - name: 'aggressive_reset1'
    type: 'clear_costmap_recovery/ClearCostmapRecovery'
  - name: 'clearing_rotation1'
    type: 'rotate_recovery/RotateRecovery'

planner_patience: 2.0 
controller_patience: 5.0
conservative_reset_dist: 3.0
planner_frequency: 5.0 
oscillation_timeout: 5.0
```

### Parameter Tuning

This configuration focuses on parameter tuning for trajectory scoring and simulation settings to optimize robot navigation. Trajectory scoring parameters, such as path_distance_bias, goal_distance_bias, and occdist_scale, influence the path planning preferences. Simulation settings include sim_time, ensuring accurate local navigation planning.

#### Trajectory Scoring

```yaml
path_distance_bias: 6000.0 
goal_distance_bias: 0.1 
occdist_scale: 0.0
forward_point_distance: 0.325
stop_time_buffer: 0.3
scaling_speed: 0.25
max_scaling_factor: 0.2
```

#### Simulation Settings

```yaml
sim_time: 4
vx_samples: 20
vy_samples: 0
vth_samples: 40
controller_frequency: 10.0
```
