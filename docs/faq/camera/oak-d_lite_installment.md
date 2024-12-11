---
title: Campus Rover Project FAQ
author: Pito Salas
date: Dec 5 2024
---

## Author
* Yutian (Tim) Fan
* Dec 5 2024
* ROS version: Noetic

## Summary

This FAQ provides answers to common questions about the Campus Rover Project, including setup, usage, and troubleshooting.

## Details

### 1. **What is the purpose of the Campus Rover Project?**
The project aims to enable a robot to autonomously map and explore an unknown environment in real time while performing tasks such as monster detection and navigation to specific goals.

---

### 2. **Which ROS packages are used in this project?**
The project relies on several ROS packages:
- **SLAM**: GMapping or SLAM Toolbox
- **Navigation**: Custom pathfinding using costmaps, Dijkstra algorithm, and pure pursuit
- **Fiducial Detection**: AprilTags (or similar)
- **Frontier Exploration**: Based on the frontier exploration code from Kai Nakamura's project.

---

### 3. **How do I set up and run the project?**
Follow these steps:
1. Launch the SLAM system:
   ```bash
   roslaunch turtlebot3_gazebo turtlebot3_world.launch
   ```
2. Start GMapping for SLAM:
   ```bash
   roslaunch turtlebot3_slam turtlebot3_gmapping.launch
   ```
3. Visualize with RViz:
   ```bash
   roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
   ```
4. Run the exploration and path pursuit scripts:
   ```bash
   rosrun frontier_exploration frontier_exploration.py
   rosrun frontier_exploration pur_pursuit.py
   ```

---

### 4. **What is the purpose of frontier exploration?**
Frontier exploration guides the robot to unexplored regions of the environment. It identifies frontiers (the boundary between known and unknown spaces) and generates navigation goals to explore these regions dynamically.

---

### 5. **How does the robot detect monsters?**
The robot uses a depth camera to detect poles wrapped with bright green tape. It spins in place to locate monsters and records their positions for interaction.

---

### 6. **What happens when the task is complete?**
Once the robot has defeated the required number of monsters, it navigates to the fiducial that marks the exit location, using its recorded position from the map.

---

### 7. **What are common troubleshooting steps?**
- **SLAM issues**: Ensure the robot has sufficient LIDAR coverage to generate a usable map.
- **Navigation issues**: Check the pathfinding logic and ensure costmaps are updating dynamically.
- **Fiducial detection**: Verify that the fiducials are well-lit and visible to the robot’s camera.

---
