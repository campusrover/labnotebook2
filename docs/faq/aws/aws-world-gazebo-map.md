---
title: How to Use AWS Robotics Maps with ROS Noetic and TurtleBot3
author: Pang Liu
type: FAQ
status: first-demo
date: Dec-2024
---
# AWS Robotics Maps Integration

## Introduction

The [AWS Robotics GitHub repository](https://github.com/aws-robotics) provides a variety of prebuilt Gazebo maps, including environments like hospitals and warehouses. These maps can be used for simulation purposes with ROS Noetic and TurtleBot3. This guide focuses on using these maps without AWS services, running simulations locally or on a lab VNC.

I recommend you try to build a local ROS environment to learn ROS, especially now ROS2 is more and more popular. Also, I recommend you to learn how to build a gazebo map, that is really interesting!

## Getting Started

Clone the AWS Robotics map repository and prepare your workspace to integrate these maps into your simulation environment.

### Cloning the Repository

1. Open a terminal and navigate to your desired workspace directory.
   ```bash
   cd ~/catkin_ws/src
   ```
2. Clone the AWS Robotics map repository:
   ```bash
   git clone https://github.com/aws-robotics/aws-robomaker-sample-application-maps.git
   ```
3. Navigate to the `maps` directory to view the available maps:
   ```bash
   cd aws-robomaker-sample-application-maps/maps
   ls
   ```

   You should see `.world` files like `hospital.world` and `warehouse.world`.

## Using the Maps in Gazebo

### Launching Maps in Gazebo

1. Open Gazebo and load a map directly. For example, to load the `hospital.world` map:
   ```bash
   gazebo ~/catkin_ws/src/aws-robomaker-sample-application-maps/maps/hospital.world
   ```
2. Verify that the map loads correctly in Gazebo. If adjustments are needed, edit the `.world` file with a text editor.

## Using the Maps with TurtleBot3

### Modifying Launch Files

To integrate AWS Robotics maps with TurtleBot3, modify the `turtlebot3_world.launch` file to use the desired map:

1. Locate the `turtlebot3_world.launch` file:
   ```bash
   cd ~/catkin_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/launch
   ```
2. Open the file in a text editor:
   ```bash
   nano turtlebot3_world.launch
   ```
3. Replace the default world file with the AWS Robotics map path. For example:
   ```xml
   <arg name="world_file" default="$(find aws-robomaker-sample-application-maps)/maps/hospital.world"/>
   ```
4. Save and close the file.

### Launching the Simulation

1. Start the simulation with the modified launch file:
   ```bash
   roslaunch turtlebot3_gazebo turtlebot3_world.launch
   ```
2. Use the keyboard teleoperation node to control the TurtleBot3:
   ```bash
   roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
   ```

## SLAM and Navigation with AWS Maps

### Running SLAM

1. Launch the SLAM node:
   ```bash
   roslaunch turtlebot3_slam turtlebot3_slam.launch
   ```
2. Visualize the map building process in RViz:
   ```bash
   rosrun rviz rviz
   ```
3. Control the TurtleBot3 to explore the map.

### Navigation

To use AWS Robotics maps for navigation:

1. Convert the `.world` file to `.pgm` and `.yaml` formats. This can be done using Gazebo and the [map\_server](http://wiki.ros.org/map_server) package.
2. Launch the navigation stack with the converted map:
   ```bash
   roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/path/to/your_map.yaml
   ```
3. Set navigation goals in RViz to test path planning and obstacle avoidance.

## Notes and Tips

- **Map Compatibility**: Ensure that the AWS Robotics maps are compatible with your Gazebo version. If models are missing, download them from the Gazebo Model Database.
- **Performance Optimization**: For complex maps, lower Gazebo’s simulation frequency to improve performance.
- **Path Planning**: Use the `move_base` node or custom algorithms to test navigation in realistic environments provided by AWS Robotics maps.

By following these steps, you can effectively use AWS Robotics maps with ROS Noetic and TurtleBot3 to simulate and test robotic applications.
