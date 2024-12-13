---
title: GestureBot
author: Jeffrey Wang, Leo Gao
date: Dec 10 2024
---
# Introduction:
Body language is an important part of human interaction and communication. Often, we rely on gestures for expression and occasionally instruction. With robotics becoming more prevalent every year and companies aiming towards producing humanoid and companion-like robots, we believed that it would be interesting to see the viability of creating such a robot. Our primary objective was to create the GestureBot, a robot that would read hand gestures and perform instructions accordingly. We aimed to give it both basic and complex instructions, and for it to perform without needing preplanned environments or mapping, so that it could simply be deployed and immediately be in use.

Our basic instructions were simple movement commands. However, for complex instructions we aimed to either have the robot follow the user or have the robot memorize object locations and traverse back to them even if led somewhere completely different. Having the robot follow a person turned out to be significantly more difficult, however, when putting obstacles into consideration and especially when other people are near the robot. Therefore, we decided to implement the latter complex instruction instead. Given that this idea was based off of the GOAT (Go to Any Thing) robot shown in class, it will be called mini GOAT for short.

# What was created:
## Gestures:
The primary feature of our robot is its ability to recognize gestures. Our original idea for implementing this was using object segmentation to find the hand, and then another algorithm for figuring out the gestures: but this would likely use up an enormous amount of processing power. After further research, we found a better solution using Google’s MediaPipe Hand Landmarker tool combined with a custom neural network made with Keras for simplicity. The hand landmarker takes in an image and produces an output of the hand’s “key points”, which are mainly the joints in each finger and the wrist of the hand. Given that this produces exactly 21 points every time it finds a hand in the image, we could create a neural network that takes in these 21 inputs and produces an output gesture based on hand training data that we could create. 


Model layers:
```
model = tf.keras.models.Sequential([
    tf.keras.layers.Input((21 * 2, )),
    tf.keras.layers.Dropout(0.2),
    tf.keras.layers.Dense(20, activation='relu'),
    tf.keras.layers.Dropout(0.4),
    tf.keras.layers.Dense(10, activation='relu'),
    tf.keras.layers.Dense(NUM_CLASSES, activation='softmax')
])
```

To obtain our training data, we filled a CSV file with a list of keypoints and a class binded to a key that we would press to add a new datapoint to the file. We did this until we had around 5000 data points. Our neural network’s layers were chosen by trial and error - we began with a neural network that had already been written in an older project written by us for classification and then changed the parameters until it worked consistently. After training the model, it can be used for gesture classification.

Logging to CSV:
```
csv_path = /csv_path
with open(csv_path, 'a', newline="") as f:
    writer = csv.writer(f)
    writer.writerow([number, *landmark_list])
    print(f"logged {number}")
```

## Mini GOAT command:
The real GOAT’s implementation involved object segmentation and depth estimation to produce an estimation of the object’s position on the map, and then used a system extremely similar to move_base to go to the object. We wanted to implement this from scratch, so we decided to first follow in their footsteps and see where we could go from there.

We found that there were three major issues:

* Accurate depth estimation is NOT possible using a single camera. 

    * Multiple different real-time monocular depth algorithms were tested with the robot (<https://github.com/nianticlabs/monodepth2>, <https://github.com/Ecalpal/RT-MonoDepth>, <https://github.com/atapour/monocularDepth-Inference> to list a few), and all of them were either too slow or infeasible for the project for various reasons.

* Object segmentation does not differentiate between unique objects.
    * If there were 20 unique books in front of the robot, it will not remember the difference between book 1 and book 20
* These algorithms are very expensive.
    * The object segmentation algorithm takes 1.5 seconds to run.
    * This was the fastest algorithm that we found that had OK accuracy.

We will address them in order, beginning with our issues with depth estimation. The original GOAT used depth estimation in order to accurately localize the object within its map. This is not possible for us because real-time monocular depth algorithms with metric depth do not exist (without taking 15 seconds to run) despite them claiming they do, such as this one by [Apple](https://machinelearning.apple.com/research/depth-pro) that claims it can create a depth map in less than one second (many users with high quality GPUs reported they could not recreate the same speed). This poses a problem because we need to find a way to record the pose of the object in order to navigate to it.

Our solution to this issue comes from the realization that the object’s pose is almost always going to be near the robot’s pose: and considering that the robot must be facing towards the object for the object segmentation algorithm to work, we can simply record the robot’s pose as the object’s pose. This allows us to travel back to the exact position where the robot sees the object, and then we can have the robot travel directly towards the object until we reach its actual position.

This poses another problem: some objects are not within the LiDAR’s range; that is, they may be smaller than the robot or above the robot. In this instance, it will not be possible to use sensor distance as a metric to determine how close the robot is to the object. This is why we must still use the depth estimation algorithm to determine an extremely rough guess of how close the robot is. While it is impossible to obtain an accurate depth reading from a monocular depth algorithm, it is still possible to get a relative reading, where closer objects will have a high value and distant objects will have a low value. Additionally, our object segmentation algorithm provides bounding boxes that indicate where the object is in our camera frame, which allows us to select just the object’s relative depth. In this way we are able to get closer to the object by getting the bounding box of the object, calculating the average of the object’s depth, travelling towards it, and then making sure our relative depth estimate isn’t too high. In the case that the object disappears from view for too long or the depth estimate goes above the threshold, we simply stop the robot because we assume either we are too close or the object is no longer in our view due to elevation or movement.

The object’s depth is calculated very simply: we take the median of all depths in the bounding box (the majority should obviously belong to the object) and filter out all depths less than the median. We are likely then left with all of the depths belonging to the object. We then take the mean of all those depths, and are left with our assumed relative distance of the object.

Sample Code:
```
box = self.predictions['bboxes'][object_prediction_index]
x_min = int(box[0])
y_min = int(box[1])
x_max = int(box[2])
y_max = int(box[3])
subarray = self.disparity_map[x_min:x_max, y_min:y_max]
median = np.median(subarray)
mask = subarray >= median
subarray = subarray[mask]
depth = np.mean(subarray)
```

Our second issue is, unfortunately, a restriction on the robot’s ability. If we wanted to add unique object mapping to the robot, we would have to add feature recognition in some way to evaluate traits of objects and assign them to some sort of dictionary or tree to “recall” previous unique objects. This would add to the already high computational cost of a depth, classification and segmentation algorithm running at the same time. So unfortunately, it will not be feasible to add this to the robot.

Our third issue can only be resolved by decreasing the callback rate of our functions. Limiting the object segmentation algorithm to one call every 1.5 seconds helps in preventing constant expensive operations. Additionally, the algorithm is only ever called when either searching for the object after navigation or when memorizing an object’s position. The depth algorithm is forced to run only when the segmentation algorithm runs, so it does not take up resources. The gesture recognition must be on constantly, but the callback rate can be reduced so that it does not become too expensive. 

We used rospy.Timer to change the callback rate:
```
rospy.Timer(rospy.Duration(1.5), self.segmentation_callback) #start object recognition callback
rospy.Timer(rospy.Duration(1.5), self.depth_callback) #start depth estimation callback
```

Now that these issues have been addressed, we can explain the full scope of the mini GOAT instruction.

There are two steps to this process: memorization and navigation.
Memorization only occurs when given a specific gesture. Upon recognition, the robot will immediately memorize eight objects within its view with the highest confidence scores, filtering out people, chairs, tables, and other common objects that are not to be remembered. It also filters out objects based on the relative depth estimates, so distant objects will not be relevant to prevent a fire extinguisher that is 20 feet away from being memorized as a bottle. After the filtering process, the remaining objects will become keys in a dictionary and assigned the robot’s pose as a value.

For the filtering process, we had to implement Non-Maximum Suppression due to the many overlapping guesses that the segmentation algorithm produced. Non-Maximum Suppression is used to output the best “bounding box” out of a set of overlapping bounding boxes; where bounding box refers to a rectangle that represents the object’s location in the frame. This is done by calculating the Intersection over Union (Area of intersection / Combined Area) of all bounding boxes and taking the highest confidence box that is above the Intersection over Union “threshold”, which determines if a box is significantly overlapping or not.

Sample Code:
```
def non_max_suppression(self, boxes, scores, threshold):
    order = sorted(range(len(scores)), key=lambda i: scores[i], reverse=True)
    keep = []
    while order:
        i = order.pop(0)
        keep.append(i)
        for j in order:
            # iou between the two boxes
            intersection = max(0, min(boxes[i][2], boxes[j][2]) - max(boxes[i][0], boxes[j][0])) * 
                            max(0, min(boxes[i][3], boxes[j][3]) - max(boxes[i][1], boxes[j][1]))
            union = (boxes[i][2] - boxes[i][0]) * (boxes[i][3] - boxes[i][1]) + 
                    (boxes[j][2] - boxes[j][0]) * (boxes[j][3] - boxes[j][1]) - intersection
            iou = intersection / union

            if iou > threshold:
                order.remove(j)
    return keep
```

# Navigation
The navigation system combines SLAM mapping and movebase. These components work together to allow a robot to autonomously explore an unknown environment, plan efficient paths, and adapt to changes in real-time. By Implementing Simultaneous Localization and Mapping (SLAM), the robot was able to build and maintain a map of an unknown environment and be able to recognize where it was located within the dynamically made map. Combined together with the built-in amcl within move base using global and local costmaps based on odom and lidar scans, it was able to effectively navgiate to saved objects.


## SLAM gmapping

In environments where no pre-existing maps are available, the robot needs to simultaneously map the surroundings and localize itself.  
SLAM solves the "chicken and egg" problem: you need a map to localize and localization to create a map.  
SLAM allows for adaptability in dynamic environments where obstacles or features may change over time.   

Purpose: To create a map of the environment while simultaneously determining the robot’s location within it.   
Importance: Maps are essential for navigation in unknown environments.   
Localization is key to determining the robot's pose relative to obstacles and goals.   

How it Works:   
The Gmapping node subscribes to sensor data (e.g., laser scans) and odometry data to update the map.   
`odom_frame`: Tracks the robot’s movement relative to the map.   
`map_update_interval`: Adjusts how often the map is updated, balancing accuracy and computation cost.   




##  Frontier Exploration

While SLAM enables mapping, the robot needs to autonomously decide where to explore next to cover the environment effectively.  
Frontier exploration identifies regions that are on the boundary between explored and unexplored areas, allowing the robot to focus on expanding the map systematically.  

Purpose: Autonomous exploration of an unknown environment.   
Importance: Automates navigation to unvisited areas by identifying frontiers which are boundaries between known and unknown regions.   
Saves manual effort and accelerates mapping of the environment.   
How it Works:   
The `explore_lite` node generates navigation goals to frontiers.   
These goals are sent to the `move_base` node for execution.  



## Costmaps

Navigation requires both global and local planning to ensure safety and efficiency:  
The global costmap is used for long-term planning and navigation across large areas.  
The local costmap is used for real-time adjustments to avoid dynamic obstacles like people or moving objects.  

Costmaps are grid-based representations of the environment, used for path planning and obstacle avoidance.  

Global Costmap:   
Purpose: Provides a high-level view of the entire environment for long-distance path planning.   
global_frame: Uses the map frame to align with SLAM-generated maps.     
static_map: Set to false in order to dynamically make the map   
Importance: Enables efficient navigation to distant goals while avoiding large obstacles.   

Local Costmap:   
Purpose: Provides a localized, real-time view for immediate obstacle avoidance.   
rolling_window: Updates the local costmap as the robot moves.   
width, height: Define the area covered by the local costmap.   
Importance: Ensures the robot avoids unexpected obstacles in its immediate vicinity.   

Common Costmap Parameters:  
Define how obstacles are detected and inflated for safety:   
obstacle_range: Maximum detection range for obstacles.   
inflation_radius: Adds a buffer zone around obstacles for safety.   
robot_radius: defines how large the robot is so it knows which path is safe and where to make it more costly.   

## Path Planning 

Robots need to compute paths that are both efficient (shortest path) and safe (avoiding obstacles). Integration of SLAM and costmaps allows the robot to dynamically adapt its path when new obstacles are detected. This is effectively and more optimally done within the movebase as it localizes itself using its odom and lidar sensors.   


Purpose: To compute and execute paths from the robot’s current position to a specified goal.   
Importance:   
Integrates global and local planning for robust navigation.    
Adapts dynamically to changes in the environment using costmaps.   
How it Works:   
Global planner for high-level paths.   
Local planner for immediate movements and obstacle avoidance.   
Configurations (base_local_planner_params.yaml):   
max_vel_x: 0.45   
min_vel_x: 0.1   
max_vel_theta: 1.0   
acc_lim_theta: 3.2   
acc_lim_x: 2.5   
These ensure smooth and safe movement by limiting velocities and accelerations.  

## Manual Waypoint Navigation  
Purpose: Allows users to manually save poses and command the robot to navigate to those poses.   
Importance:   
Used for our gesture recognition to remember a specific pose when it recognizes an object and be able to navigate back to it.   


# Usage

To run the robot, first bring up the master node on the robot.
Then do ```cd movebase/launch``` and run ```roslaunch move_base.launch```.
After, do ```cd real``` and run ```python3 roscam.py```.
We only use an image subscriber and publisher for debugging.


# Story:
Neither Jeffrey nor I have ever been a pet owner of dogs and so we thought it would be really interesting if we were to create a robot that could follow gestures since it wouldn’t really be able to listen to our voices. 

The initial outline for the project was to use neural networks combined with google media pipe to recognize and train gestures and gesture recognition. Then, it would perform pre programmed commands such as to move around, spin, or to go back to an original position.

However, it wouldn’t have the self navigating aspect that an actual dog would have. 

This gave us the idea to implement the GOAT (go to any thing) from class, because it was interesting and it would also be able to replicate fetching. This required us to have object recognition, depth recognition, and SLAM. After deciding on doing these implementations, we needed to do extensive research on what we could do to reduce overhead on the bot since if it was too demanding many errors could occur while running. 




## How it unfolded

The GestureBot project began with a straightforward idea: combining gesture recognition with basic movement commands to emulate a robot that responds to human gestures. The initial inspiration stemmed from the desire to create a robot with functionality akin to a pet—intelligent and interactive but devoid of auditory command reliance. Jeffrey and I both lacked firsthand experience with pets, particularly dogs, which made the concept of creating a robot capable of "fetching" or "following" based on gestures an exciting challenge.

We initiated the project with an extensive research phase. Gesture recognition stood out as the project's backbone, so we explored various methodologies, including Leap Motion sensors. However, after discovering that the Leap device we had access to was incompatible due to proprietary changes, we pivoted to a solution involving Google MediaPipe. This tool, combined with a custom neural network built using Keras, enabled efficient and lightweight gesture classification.

Once gesture recognition was operational, we expanded the project's scope to include elements inspired by the GOAT ("Go to Any Thing") system from class. This involved implementing advanced features like object recognition, depth estimation, and autonomous navigation. At this point, we divided responsibilities: Jeffrey focused on object recognition and depth estimation, while I worked on live mapping, localization, and path planning. This division of labor allowed us to address the project's complex requirements in parallel.


Our teamwork was characterized by clear communication and mutual respect for each other's expertise. Although we worked on separate components, we regularly synchronized our efforts to ensure seamless integration. Debugging sessions became collaborative problem-solving exercises, where one person's fresh perspective often revealed overlooked issues.

This collaboration extended beyond technical implementation. For example, Jeffrey's insights into reducing computation overhead significantly influenced my approach to SLAM and costmap configurations. Similarly, my work on creating a modular navigation framework informed Jeffrey's decisions regarding depth estimation and object segmentation integration.

## Problems That Were Solved:
Gesture Recognition and Data Collection:
Initial attempts at gesture recognition revealed a need for consistent and comprehensive training data. Using Google MediaPipe's keypoint detection, we logged thousands of datapoints in a custom CSV file to train a neural network. This required refining our approach to data labeling and augmentation to improve accuracy.

Depth Estimation Challenges:
Depth estimation emerged as a major hurdle due to the limitations of monocular camera systems. After testing multiple algorithms, we realized that accurate metric depth could not be achieved in real-time. Our workaround involved recording the robot's pose as a proxy for the object's location and using relative depth estimation to refine navigation.

Resource Management:
The computational demands of running multiple algorithms simultaneously posed a risk to system stability. We optimized callback rates for object segmentation and depth estimation, reducing resource usage without compromising functionality. This adjustment was crucial for maintaining smooth robot operations.

Object Recognition and Differentiation:
The segmentation algorithm's inability to distinguish unique objects led us to implement Non-Maximum Suppression (NMS) to refine bounding box selections. Although true object differentiation remained beyond our scope, the NMS-based filtering significantly improved recognition reliability.

SLAM Integration:
Integrating SLAM with gesture-based navigation required careful tuning of parameters like obstacle inflation and robot dimensions. By balancing accuracy and computational efficiency, we ensured that the robot could navigate dynamic environments without frequent errors.

## Pivots
Transitioning from Leap Motion to MediaPipe for gesture recognition due to hardware limitations since leap was purchased by another company and the old software no longer worked.
Added a monocular depth estimation algorithm to detect how close the robot is relative to the object in the final stages of going back to the object so it is able to "fetch" more accurately.

Also, the original idea was to create our own slam algorithm in which we created our own Frontier search algorithm, which looked for frontiers,the boundary points, using bfs on the map. Within frontier search we originally had to create our own occupancy grid and custom messages to publish, however, this lead to all sorts of overhead which is one of the main reasons we had to pivot. Path planner, which used A* to plan paths. Pure pursuit, which would follow the path using a point slightly in front of the robot, and frontier exploration which coordinated these three classes. \
However, we found that this caused the robot to malfunction and have increased latency because of the amount of overhead and it would only work sometimes. This caused us to pivot to using movebase with slam gmapping and explorelite nodes with yaml costmaps because it was more optimally written and created less overhead in the robot. Slam gmapping used lidar and odom to keep track of where the robot was relative to its surrounding as well as creating the map and by combining that with the explorelite node it would know where is the least explored area which allows the robot to active search for more of the map autonomously. Essentially the robot greedily search for all frontiers and can be used or not used based on the gestures. 

## Self Assessment
The project achieved its goals of gesture recognition and autonomous navigation to an object. We did not expect the cluster to be able to handle multiple heavy algorithms at the same time, so being able to successfully run object segmentation, depth estimation and gesture recognition at the same time on top of a ROS navigation stack is quite unexpected. Most of the time, a lot of issues occur when translating outputs between algorithms, so it was very pleasing to see that most of the data was actually extremely organized and very easy to both use and interpret. Because of this, the way that the algorithms interact is actually quite smooth. Overall, given our limitations and workarounds, we are happy to see that the robot works and functions as intended. 

GestureBot represents a synthesis of cutting-edge techniques in gesture recognition, object segmentation, and autonomous navigation. By tackling real-world constraints with creative solutions, we transformed an ambitious idea into a functional prototype. Beyond the technical achievements, this project underscored the value of teamwork, adaptability, and the iterative design process in robotics development.


