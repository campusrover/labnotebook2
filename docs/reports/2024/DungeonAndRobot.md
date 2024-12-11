---
title: Campus Rover Project
author: Pito Salas
date: Dec 5 2024
----------------

## Introduction

### Problem Statement

The objective of this project is to enable a robot to autonomously map an unknown environment in real time without relying on a pre-existing map. While mapping the environment dynamically, the robot must also explore it autonomously, identifying and interacting with specific elements like frontiers and monsters. This necessitated the use of SLAM (Simultaneous Localization and Mapping) combined with frontier exploration.

### Relevant Literature

The concept of frontier exploration and its implementation were sourced from Kai Nakamura's project on autonomous mapping robots, as detailed on the webpage: [https://kainakamura.com/project/slam-robot](https://kainakamura.com/project/slam-robot). Nakamura’s explanation of frontier exploration and associated code served as the foundation for integrating autonomous exploration in this project.

### Team Members

- **Yutian (Tim) Fan**
  - Email: [example@domain.com](mailto\:example@domain.com)

## What Was Created

### Outline

#### 1. Key Components

1. **Frontier Exploration**:

   - **Frontier Detection**: Identifies unexplored areas (`-1` in the map) adjacent to free space (`0`).
   - **Pathfinding**: Uses a combination of:
     - **Costmap**: Represents proximity to obstacles, ensuring safer navigation.
     - **Dijkstra Algorithm**: Calculates the shortest path to the target (monster or frontier).
     - **Pure Pursuit**: Ensures smooth execution of the path by adjusting the robot's trajectory dynamically.

2. **Monster Interaction**:

   - Combines visual detection, navigation, and physical interaction.
   - Operates in tandem with frontier exploration.

3. **Task and Exit Management**:

   - Switches the robot's behavior dynamically based on task status:
     - **Exploring**: Continue frontier-based navigation if monsters remain.
     - **Exiting**: Navigate to the fiducial once the task is complete.

#### 2. Background Tasks

1. **SLAM**:

   - Use a SLAM algorithm (e.g., GMapping or SLAM Toolbox) to generate an `OccupancyGrid` map.
   - The `/map` topic provides the real-time map.

2. **Fiducial Detection**:

   - Detect fiducials to identify the exit location in the same fashion as the fiducial PA.

#### 3. Monster Interaction

1. **Locating Monsters**:

   - Spin the robot in place to locate monsters using the depth camera.
   - Identify monsters based on bright green tape wrapped around a pole.
   - Record the positions of detected monsters.

2. **Navigating to Monsters**:

   - Move to the location of a detected monster using the shared pathfinding approach.

3. **Knocking Over Monsters**:

   - Physically knock over the pole by colliding with it.
   - Use velocity commands to navigate the robot into the monster's position.

#### 4. Exploration Cycle

1. **Monster Hunting**:

   - Detect monsters by spinning in place and scanning the environment.
   - Navigate to each detected monster and knock it over.
   - Ensure all known monsters in the current region are cleared.

2. **Frontier Exploration**:

   - Identify frontiers (unexplored regions) using frontier detection.
   - Navigate to the nearest frontier and expand the map.

#### 5. Task Management

1. **Task Completion**:

   - Track the number of monsters defeated and determine when the task is complete.
   - Use a counter to track defeated monsters and verify goal completion.

2. **Exit Navigation**:

   - Use the recorded position of the fiducial as the goal.
   - Plan a path and navigate to the exit using the shared pathfinding approach.

### Guide on How to Use the Code

#### Commands to Execute

To execute the project setup:

1. Start the SLAM system:

   ```bash
   roslaunch turtlebot3_gazebo turtlebot3_world.launch
   ```

2. Launch GMapping for real-time SLAM:

   ```bash
   roslaunch turtlebot3_slam turtlebot3_gmapping.launch
   ```

3. Launch RViz for visualization:

   ```bash
   roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
   ```

4. Run the frontier exploration script:

   ```bash
   rosrun frontier_exploration frontier_exploration.py
   ```

5. Run the path pursuit script:

   ```bash
   rosrun frontier_exploration pur_pursuit.py
   ```
