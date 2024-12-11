

> CS119A - Autonomous Robotics - Professor Pito Salas - December 10, 2024

 - Daphne Pissios
 - Chloe Wahl-Dassule
 - Jungju Lee

# Final Report - Hide and Seek
## Github Repository
https://github.com/AnotherDaphne/robots_hide_and_seek

## Introduction

Although there are many different tasks that robots can perform, arguably all of these tasks involve the actuation of the robot or sensors of the robot. After going through the current curriculum in the Autonomous Robotics course, a project that our group decided that can best demonstrate the utilization of actuators and sensors was to perform a Hide and seek game.

Hide and seek game consists of two separate teams; Hiders and Seekers. Hider's main objective is to find the best hiding spot that will prevent them from being found for the longest time. There is no limit to using sensory devices but there is a limit to actuators, specifically motor speed. There also is no limit to using any ROS topics. Same rules apply for Seekers. The Seeker's main objective is to find the Hiders. To better detect Hiders, Hiders will have fiducials attached on all four sides of the robot. This imposes one additional mandatory task for seekers which is having to seek for fiducials at all times.

To better demonstrate the development of hide and seek algorithms, final deliverables will consist of video that shows how algorithms have been making improvements; multiple generations of algorithms based on trial and failure. As per this paper, however, there will only be initial proposals of each hide and seek algorithms by participants.

## Rules

Following are basic rules that will be imposed during the hide and seek game.  
- All robots have a limit to x coordinate linear velocity at 0.5 on code.  
- All robots have a limit to z coordinate angular velocity at 0.5 on code.  
- Under move_base topic operation, no velocity limit is imposed.  
- Once the game starts, Hiders will have 5 minutes of hiding time. Then, seekers will start searching.

## Environment Control

The environment that our team has been assigned to perform the hide and seek had multiple objects that would interfere with the Lidar sensors; chairs, bricks with holes, round trash bin, or a table that has an empty space just enough for the robot to pass through. To control this environment, cardboard boxes, wooden walls were used to block under the desk, and hide any potential interference.

Although the room was already in a very unique shape which provides a lot of blind spots for hiders, our team has decided that the room was narrow and did not consist of enough variables. To provide more options for robots to hide, artificial pockets were created. These pockets were located both in the middle of the room, or tangent to a wall.

# Seeker Algorithms and Implementations - James
## Guide on how to use the code
1. Run ```roslaunch <package-name> fiducial.launch```
2. Run ```rosrun <package-name> mapper.py```
3. Search Algorithm ( either one )
3-1. Run ```rosrun <package-name> random_search.py```
3-2. Run ```rosrun <package-name> wall_follow_search_v0.py```

## Fiducials

Since computer vision was a very sophisticated and niche feature, it was not yet to be implemented for searching other robots. One easy workaround was to attach fiducials on all four sides of the robot, then treat the hider to be 'found' once the seeker detects the fiducial. If multiple hiders were to perform hiding at the same time, this would reduce the confusion as unique fiducials would allow the seeker to distinguish between two turtlebots that looks nearly identical.

Aruco D7 Fiducials were attached to all four sides of the robot. Initially, these fiducials can easily be detected once 'aruco_detect' ros-package was launched. List of fiducials to look for was detected before seeking starts. From the buffer of tf messages, fiducials message was seeked and once specific fiducial was found, the hider is considered 'found' and the seeker removes the 'found' fiducial from seeking list and continues searching if there is any more hiders to find.

## Algorithms - (Pseudo) Random Search

This random search algorithm is based off of the following logic; When playing hide and seek in person at a completely random location, the most reasonable thing that one does is moving around to a random location, hoping to find hiders. However, this algorithm consists multiple problem. As the randomness of this algorithm suggests, it could be highly inefficient and run into a problem where the algorithm runs unluckily, and robot does not reach the other side of the room, or fails to look for one specific corner even after a long random search. To prevent this, the algorithm will not be completely random but rather make decisions based off of the lidar information.

From 0 degree to 360 degree, robot will fetch Lidar distance information with 30 degree interval, being total of 12 candidates. Then, out of 12 candidates, robot will randomly select one direction out of three directions that has the longest distance read. Then, robot will turn to this direction with -15 to 15 degree offset that is randomly selected.

After direction has been determined, the robot will perform a linear movement. This movement will also be randomly decided between 50% and 90% approach to the wall; time that it performs linear movement will be proportional to its distance from the wall.

## Algorithms - Wall Follow Search 

This algorithm is based off of the following logic; To search the entire room visually, every single blind spot will be visited if one follows along the wall. This algorithm is rather simpler than Random Search. Robot will perform a wall follow mainly utilizing its Lidar sensor. While performing a wall follow, searching for fiducial will not stop. Robot will also periodically stop, turn around to scan the whole room for fiducial scan; this is regarding the pockets in the middle of the room or missed blindspots that could be behind the robot.

There is a lot more leniency than having to perform an actual wall follow. As the robot has to visually scan the whole room, we can allow robot to perform wall search with some leniency to its distance from the wall. This should be fine tuned, although, as there is a chance that robot runs into a pocket and considers it as a wall, causing problems with the wall follow itself.

There also is a lot to be optimized for this algorithm. For example, the OV5647 camera module that is used for Raspicam, which is attached to the turtlebot has default of 54 degree of horizontal field of view. This means when the robot is parallel to the wall, 27 degree to the left of the robot is already being scanned. When robot decides to turn around for full room scan, it has to only turn 153 degree instead of 180 degree.

Also, depending on the size of fiducial, the capability of camera to detect fiducial could vary. Once measuring the maximum fiducial detection distance, robot could use it as an interval. For example, if the maximum fiducial detection distance is 3 meters, robot can stop after every 6 meters travel, do a full room search. This will cover circular area with diameter of 6 meters.

# Hider Algorithms and Implementation - Chloe

## Guide on how to use the code

1.  Robot must have AMCL running using the following command
    

1. ``` roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:="filename"```
    

2.  Give robot accurate starting point
    
3.  Run``` hider_real.py``` or ```hider_sim.py```
    
4.  Run ```scan_sim.py```
    
5.  Run ```timer.py```
    

*Files must be run in this order*

## Algorithm overview

This algorithm works by attempting to recognize pockets, small areas walled in on 3 sides, in LiDAR data, calculating coordinates to the pockets, assessing the best pocket, and navigating to the coordinate at the end of the hiding time.

## Recognizing pockets - scan_sim.py

As the robot is moving around the space, it will be regularly checking the data from LiDAR and looking for a group of points that look like pockets. The diagram on the right shows what pockets look like in the data. The algorithm is a large set of if statements that comb through the points looking for local minima, maxima and plateaus. I had to rewrite this section many times as my understanding of how the pockets showed up on the LiDAR evolved. I eventually settled on looking for points that were all close together, started with an increase, had a maximum, although there could be multiple points at the same level of the y axis and those would all be counted as the maximum (added to the plateau array), and ended with a decrease. For the most part, this technique does identify pockets in the LiDAR data though it will also identify areas that are incorrect. I manage that at a later point in the algorithm.

## Calculating the coordinates - scan_sim.py

Once the pockets are identified, the algorithm calculates where on the map the pockets are. This is done through basic trigonometry. First the algorithm gets the first recorded localmin_1 the center of plateau and the last recorded localmin_2. Then it finds the coordinates of these points. It does this by getting the robot's coordinates and yaw from get_robot_position() and the distance reading from the LiDAR. Using simple trigonometry, the coordinates of the points are identified. Then the centroid coordinates are calculated and published. This was the most challenging part of the project for me because I was incorrectly trying to use the degrees from LiDAR as the angle for the trigonometry functions without accounting for the yaw of the robot. This meant that everytime I calculated the coordinates, they were always nearby but rotated incorrectly. This bug took me a long time to track down and understand. However, once I figured it out I was able to get accurate coordinates and publish them.

## Ranking pockets - hider_sim.py or hider_real.py

Once the coordinates are received they are screened against previously visited coordinates to avoid revisiting the same places. Coordinates are only added to the queue of places to visit if they are not within the set radius of any points that have already been visited, or points that are currently in the queue. Coordinates are also not added if they match any that are in the preempted array but they can be nearby those points.

When a coordinate is visited, assuming the goal isn’t preempted, the algorithm takes the average of all the LiDAR data as a simple measure of how enclosed the space is. This number is stored along with the coordinate.

As previously mentioned, the pocket detection does make mistakes and will send the robot to places that are incorrect. Additionally, sometimes calculated coordinates are in a wall or close enough to a wall that move_base cannot navigate to it. In order to minimize the time spent on these goals, I implemented a time limit that uses odometry to see how much the robot has moved in the last publication. I have two tiers, one that checks if the robot has not moved at all and one that checks if the robot has moved a small amount (actual values differ between sim and real). If the robot has moved too little and triggers these conditions, the goal is preempted and the robot moves on to the next goal. This decision is purely strategic for the hide and seek game. I found that the recovery behavior of move_base took far too much time and was triggered a lot, meaning that the robot would spend well over a minute trying to navigate to a difficult coordinate instead of continuing to explore. Ultimately, it was more effective and efficient for the robot to give up on that specific coordinate, hoping that the pocket would be detected again, and a better coordinate would be calculated. This is the part of my algorithm I am least happy about. With unlimited time I would continue to work on this problem and try to find a more elegant solution.

## Game logic

The game logic of my program is very simple, once the timer is running, the robot begins to move to its first coordinate which is always 0,0 if possible. I did this because the center of the map should give the robot the most starting options in the most number of situations. As long as there are pockets in the queue, it will continue to go to those. As the robot is continuously looking for pockets the queue is rarely empty. Once 5 minutes have passed, the algorithm loops through all the successful visited spots and sorts them based on which one had the lowest average of all LiDAR data. Then it will move there in the remaining minute and end the program.

# Hider Algorithms and Implementation - Daphne

To run this code

1.  ```roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:="filename"```
    
2.  Run the ```hider_explored.py``` file
    

The script defines a Hider class that creates a robot navigation strategy designed to explore an environment and ultimately hide in a corner within a 5-minute time limit. Using 3 states: “explore”, “follow_wall”, “hide”, in order to guide the robot to its destination.

The exploration phase begins with the robot waiting to receive map data and then moving around the environment. During this initial phase, the robot will move to a random location on the mpa. The goal is to locate potential walls that can be followed towards a corner. The robot has a built-in time limit for this initial exploration stage.

When a wall is detected during exploration, the robot transitions to a "follow_wall" state, which is managed by a PID controller. The PID helps the robot follow the wall smoothly. The wall-following uses sensor data from the LiDAR to adjust the robot's movement, attempting to stay roughly one meter from the wall while moving forward.

As the robot follows the wall, it continuously checks for corner detection. A corner is identified when the LiDAR detects very short distances on both the left and right sides of the robot, indicating it has approached a corner of a room or enclosed space. Once a corner is detected, the robot transitions to the "hide" state, where it goes farther into the corner and then stops moving to effectively "hide".

If the robot is in the “follow_wall” state for too long, it returns to the explore phase for some time to try and find a fall with more potential places to hide.

The entire program is constrained by a 5 minute deadline. If the robot fails to find a good hiding place, it will stop moving wherever it is, making it a lot easier for the hider to find it if the robot cannot complete its goal in time.  
  

A challenge that has come up during the development of this program has been the corner detection, and the robot’s positioning to hide. I have been using the LiDAR to detect the corners, but the detection can be imprecise. Getting the robot into the corner it has decided to hide in can also be difficult. Not only are some of the hiding spaces in the environment relatively small compared to the size of the robot, the calculation of getting the robot hidden sufficiently within the corner can also be imprecise and may result in the robot being positioned farther away from the hiding corner than is practical for a hide and seek game.

# Challenges and Hardships

As this is a game that is performed by multiple robots, our main hardship was that we could not perform this in a Gazebo simulation. This also caused some problems with imposing time limits or deadlines to certain team members, causing delays to deliverables.

A challenge within gazebo was that only certain maps worked when trying to create maps with move_base. Because of this issue, it was more difficult to test the algorithms in diverse environments to see if any issues in a real life scenario would arise

We also faced issues with testing in the lab due to not having enough turtlebots available. We originally wanted to have three turtlebots, each running one of the programs, in the room at the same time. However, because there were never three turtlebots available at once, we modified our project to work with one turtlebot. After each hider algorithm has finished, instead of leaving the turtlebot there, we put a block with a fiducial on it to save its place. This way the turtlebot was available for the next hider algorithm. This meant that the seeker was looking for fiducials on blocks instead of on turtlebots.

# Source Files

## James
| File name | Description | Subscribers | Publishers | 
|--|--| --| --|
| /launch/fiducials.launch | Launch file that runs ***aruco detection***. Must be run before running any other python files. |
| /src/mapper.py | Mapper for Fiducials. Must be run before running seeker algorithm. Searches the fiducual from tf2_buffer and published the pin through the broadcaster topic. |tf2_buffer | tf2_broadcaster |
| /src/random_search_v**0**.py | Multiple versions of random_search algorithms. Relies on pre-programmed pseudo random decision trees.| tf2_buffer, /odom(Odometry), /scan(LaserScan) | /cmd_vel(Twist)
| /src/wall_follow_search_v**0**.py | Multiple versions of wall_follow_search algorithms. Relies on wall follow algorithm modified with certain search action per interval.| tf2_buffer, /odom(Odometry), /scan(LaserScan) | /cmd_vel(Twist)

## Chloe
| File name | Description | Subscribers | Publishers | 
|--|--| --| --|
|  hider_real.py| Filters coordinates received from _my_scan_ subscriber, handles all movement through move_base, and contains all game logic | my_scan (Float32MultiArray), my_timer (Int32), /scan (LaserScan), my_odom (Point), odom(Odometry) | cmd_vel (Twist)|
| scan_sim.py| Reads lidar data and detects pockets. Calculates coordinates of pockets and publishes them to _my_scan_|/scan (LaserScan) |my_scan (Float32MultiArray) |
| timer.py |Helper program that publishes an increasing integer at the set rate to _my_timer_ | | my_timer (Int32)|
| my_odom.py| Helper program that handles the robots odometry and publishes data to _my_odom_ | odom(Odometry) | my_odom (Point)

## Daphne
| File name | Description | Subscribers | Publishers | 
|--|--| --| --|
| hider.py | Entire hiding algorithm | /map, /scan (LaserScan) | cmd_vel (Twist) (move_base) |
