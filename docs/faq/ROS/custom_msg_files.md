# Installing and Building Custom Message Files in ROS

Harry Yu

## Overview

In ROS, custom message files allow you to define your own message types for communication between nodes. This guide will walk you through the process of creating, installing, and building custom message files in a ROS package.

## Step 1: Define Your Custom Message Files

1. **Create a `msg` Directory:**

   Inside your package, create a directory named `msg`:

   ```bash
   cd ~/catkin_ws/src/my_custom_msgs
   mkdir msg
   ```

2. **Create Message Files:**

   Inside the `msg` directory, create your custom message files. For example, create a file named `MyMessage.msg`:

   ```
    int32 round_time_remaining
    string game_phase
    bool bomb_planted
    geometry_msgs/Point bomb_location
    string[] dead_players
   ```

## Step 3: Modify `CMakeLists.txt`

Edit the `CMakeLists.txt` file in your package to include the message generation dependencies:

1. **Find and uncomment/add the following lines:**

   ```cmake
   find_package(catkin REQUIRED COMPONENTS
     std_msgs
     message_generation
   )
   ```

2. **Add your message files:**

   ```cmake
   add_message_files(
     FILES
     GameStateMsg.msg
   )
   ```

3. **Generate messages:**

   ```cmake
   generate_messages(
     DEPENDENCIES
     std_msgs
   )
   ```

4. **Include `message_runtime` in `catkin_package`:**

   ```cmake
   catkin_package(
     CATKIN_DEPENDS message_runtime
   )
   ```

## Step 4: Modify `package.xml`

Edit the `package.xml` file to include the message generation dependencies:

1. **Add the following dependencies:**

   ```xml
   <build_depend>message_generation</build_depend>
   <exec_depend>message_runtime</exec_depend>
   ```

## Step 5: Build Your Package
    ```bash
    ws
    catkin_make
    ```

