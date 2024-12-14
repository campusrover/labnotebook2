# Counter-Strike Bot

[Github repo](https://github.com/HandsomeHarry/cs_bot)

Harry Yu, Zared Cohen, TsonOn Kwok

## Introduction

Our goal was to create a robot simulation of the popular game franchise, Counter Strike, which is a first-person shooter game where teams of terrorists (Ts) and counter-terrorists (CTs) fight for their specific goals. The Ts must plant a bomb and defend it until detonation, and the CTs must eliminate all Ts or defuse the bomb if planted. If the round time runs out before either goal occurs, the CTs will be victorious. For our implementation, we created a 2v2 version of this game in a simulated environment (gazebo), and a 1v1 version in the real world. We also designed different personalities and guns for each robot (player), which provides a more realistic simulation of the game with an unpredictable outcome.

### Problem statement (objectives)
- Robots can detect enemy teams and shoot 
- Robots can successfully navigate the course and avoid obstacles/other robots
- Server node manages player health, game state, bomb site locations
- Communication between local (robot) nodes and server node 

## What was created
1. There are a few groups of tasks we had to work on:
the gazebo world simulation
2. The creation of the map using SLAM, how bomb site locations, patrol points and spawn points are defined. This must be done manually.
3. AMCL that allows the robots to navigate and avoid obstacles
4. Personalities of robots, robot actions when situations change using FSMs (very complicated)
5. Modularized launch files to be used when needed

### Files Overview

| **File/Component** | **Description** |
|-------------------|-----------------|
| **world3.world** | A gazebo world map of a simplified version of Inferno A site in Counter-Strike. |
| **publish_point.py** | Tool for generating map configuration points. Subscribes to /clicked_point. Records clicked points in RViz for map setup, saves spawn points and bomb site locations to YAML config file, launches map server and RViz for point selection. Used during the map setup/configuration phase. |
| **map_manager.py** | Loads and manages map configuration from CSV files. Node: map_manager. Publishes to /game/map_markers and /game/map. Handles bomb site locations and spawn points. Publishes visualization markers for map features. Creates RViz markers for bomb sites and spawn locations. Manages map coordinate system and point conversions. |
| **game_manager.py** | The central 'server' node that manages the game state. Handles round timers, bomb events, and robot health. Publishes game state updates to all robots. Manages combat events and damage calculations, tracks dead players and round winners. Controls round phases (PREP, ACTIVE, BOMB_PLANTED). |
| **robot_controller.py** | Base class for all robot behaviors. Handles movement, combat, and state management. Processes visual detection of enemies. Manages navigation and pathfinding. Interfaces with ROS navigation stack. Parent class for personality-specific behaviors. |
| **CT_normie.py, T_aggressive.py, etc.** | Child classes of `robot_controller.py`. Inherits all combat management and specific behavior, and uses FSMs to manage different states for different behaviors and teams. |
| **gun.py** | Defines weapon types and their characteristics (rifle, sniper, SMG). Manages weapon properties like damage, fire rate, and accuracy. Handles ammo management and reload timers. Provides shooting mechanics with accuracy calculations. Returns damage values based on successful hits. |

Topics published:

| **File/Component**   | **Type**     | **Topic**                  |
|----------------------|--------------|----------------------------|
| **map_manager.py**   | Publishes    | /game/map_markers          |
|                      | Publishes    | /game/map                  |
| **game_manager.py**  | Publishes    | /game/state                |
|                      | Subscribes   | /game/robot_states         |
|                      | Subscribes   | /game/shoots               |
|                      | Subscribes   | /game/bomb_events          |
| **robot_controller.py** | Publishes | /game/robot_states         |
|                      | Publishes    | /{robot_name}/cmd_vel      |
|                      | Publishes    | /game/shoot                |


## How to Run the Game (in Gazebo)

### Building the Environment
---
1. Ensure the `transitions` library is installed:
   ```
   pip3 install transitions
   pip3 install pandas
   ```
2. Navigate to your catkin workspace:
   ```
   cd <your-catkin-workspace>
   ```
3. Build the workspace:
   ```
   catkin_make
   ```
   If no error messages appear, proceed to the next step.

### Generating the Map
---
We need a robot to explore the map using SLAM to manually generate the world map for later use.

1. **Launch SLAM** using `gmapping` by running the following command:
   ```
   roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
   ```

2. Open a new terminal and control the robot with the keyboard:
   ```
   roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
   ```
   Drive the robot around the target area to scan its surroundings.

3. Once the entire target area has been scanned, open another terminal and save the map:
   ```
   rosrun map_server map_saver -f `rospack find cs_bot`/maps/<map_name>
   ```
   This will generate two files: `<map_name>.yaml` and `<map_name>.pgm` in the `cs_bot/maps` directory.

### Defining Areas
---
We will define the bomb sites using our script and `rviz`.

1. Run the following command:
   ```
   rosrun cs_bot publish_point.py
   ```

2. Click on the map according to the prompt. The results will be saved to `/config/points.yaml`.

Then run `rosrun cs_bot launch.py` to load the yaml file into the launch file. 
The coordinates saved in `/config/points.yaml` will also be used by other nodes later.

![](../../images/csbot/slam.jpg)


### Launch Game
---
Run the game by launching sim_2v2.launch, which launches all the nodes required including launching 4 robots and starting the game server node:
```
roslaunch cs_bot sim_2v2.launch
```
This launch file calls a few other launch files and sets up a 2v2 environment.
![](../../images/csbot/robot.jpg)

Below is an image of the game running, with the T robots moving to site and planting, and the CT robots defending and dealing damage to robb(T) bot and a UI interface written in Qt.
![](../../images/csbot/1.png)
Here is the multi-robot AMCL working:
![](../../images/csbot/3.png)

## Story of the Project:
This was an extremely ambitious and challenging project that required the team to pivot multiple times to achieve successful completion. Our initial concept was to create a complex 2v2 robot gunfight game, with a server node managing critical elements such as health, player location, game state, round timer, alongside player-side hit detection to simulate real-world game mechanics. While the idea appeared straightforward in theory, implementing server-based gunfight mechanics proved unreliable, with inaccuracies in hit detection leading to significant challenges. This prompted our first pivot: shifting from server-based hit detection to relying solely on the shooter robot’s perception of its enemy for accuracy.

Implementing shooting mechanics was another major hurdle. The robots' cameras were mounted on their undersides, which complicated aiming while moving. Adjustments required the gun's angle to align dynamically with the target, introducing numerous variables, such as gun_angle, enemy_to_center_angle (calculated using OpenCV), and current_speed. These complexities made fine-tuning exponentially harder. To address this, we simplified the shooting mechanic by fixing the gun’s aim to the robot’s center of view. While this reduced the computational burden, it required robots to remain stationary while shooting—a limitation reminiscent of the strategic elements in the original Counter-Strike game.

Despite delays in real-life deployment, we believe the project achieved a high success rate, and the demo's performance exceeded our expectations. Throughout this journey, we gained valuable insights into SLAM mapping, the nuances of AMCL, the limitations of the ROS system, server-client communications, multiple complex FSM implementation for different player personalities, server-side data management and resource allocation, defining and building custom message types, reading and writing to launch files directly, fine-tuning move_base parameters, optimizing color detection, and managing Gazebo physics challenges. However, the most significant skill we developed was debugging and problem-solving. Debugging accounted for approximately 70% of our total project time due to the ambitious nature of the project and the meticulous implementation of its various components. We wish for more time to work on the project to make it perfect, but we are satisfied of what we created as is.
