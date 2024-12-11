---
title: Setting the Camera Parameters
date: Dec-2024
author: Zixin Jiang
status: new
---
# Setting the Camera Parameters

by Zixin Jiang

This is a quick guide to set the camera parameters such as **contrast** or **brightness**. This is particularly helpful for improve callback image quality.

## Prerequisite

- A robot with a camera
- VNC

## Basic Steps
### Get the appropriate value
1. Connect and launch your robot. 
2. Once your robot is ready, open vnc and run ```rqt```. You can adjust the parameters with sliders in this window.
3. In a seperate terminal, run ```rqt_image_view```. This will shows the camera call back image with any updated parameters. 
4. However, any modification on parameters using this method is not going to save to robot. Thus, all the changes will be set back to default when you launch the robot again.
5. Record all the changes you make, including the parameter name and value, such as "contrast" and "73".
### Store the change to launch file.
1. Connect to your desired robot, but don't bring it up. Locate to the launch file directory in the onboad terminal with following command:
```
roscd turtlebot3_bringup/launch
```
2. Copy the .launch file we usually use for bringup the robot with comment 'cp'.
3. Open the new .launch file (let's call it new.launch), and modify the following lines: Add ```<arg name=”contrast” value=”73”/> ``` between ```<include>``` block. Change the **contrast** and **73** to the parameter and value you want.
```
<include file="$(find raspicam_node)/launch/camerav2_410x308_30fps.launch">
    <arg name=”contrast” value=”73”/> 
</include>
```
4. Save the .launch file.
5. Nevigate to camera launch folder with the follow command
```
roscd raspicam_node/launch/
```
6. Since from the previous launch file we find that the camera is launched with file "camerav2_410x308_30fps.launch", we open this file and add the following change:
```
<arg name="contrast" default="50"/>
```
7. And under the ```<node type="raspicam_node">```, we add the following line:
```
<param name="contrast" value="$(arg contrast)"/>
```
8. Finally, launch the robot with the new launch file instead of the default one.
```
roslauch turtlebot3_bringup new.launch
```