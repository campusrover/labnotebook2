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





```csv_path = /csv_path
           with open(csv_path, 'a', newline="") as f:
               writer = csv.writer(f)
               writer.writerow([number, *landmark_list])
               print(f"logged {number}")
```

## Mini GOAT command:
The real GOAT’s implementation involved object segmentation and depth estimation to produce an estimation of the object’s position on the map, and then used a system extremely similar to move_base to go to the object. We wanted to implement this from scratch, so we decided to first follow in their footsteps and see where we could go from there.

We found that there were three major issues:
Accurate depth estimation is NOT possible using a single camera. 
Multiple different real-time monocular depth algorithms were tested with the robot (https://github.com/nianticlabs/monodepth2, https://github.com/Ecalpal/RT-MonoDepth, https://github.com/atapour/monocularDepth-Inference to list a few), and all of them were either too slow or infeasible for the project for various reasons.
Object segmentation does not differentiate between unique objects.
If there were 20 unique books in front of the robot, it will not remember the difference between book 1 and book 20
These algorithms are very expensive.
The object segmentation algorithm takes 1.5 seconds to run.
This was the best and fastest algorithm that we found.

We will address them in order, beginning with our issues with depth estimation. The original GOAT used depth estimation in order to accurately localize the object within its map. This is not possible for us because real-time monocular depth algorithms with metric depth do not exist (without taking 15 seconds to run) despite them claiming they do, such as this one by Apple https://machinelearning.apple.com/research/depth-pro that claims it can create a depth map in less than one second (many users with high quality GPUs reported they could not recreate the same speed). This poses a problem because we need to find a way to record the pose of the object in order to navigate to it.
Our solution to this issue comes from the realization that the object’s pose is almost always going to be near the robot’s pose: and considering that the robot must be facing towards the object for the object segmentation algorithm to work, we can simply record the robot’s pose as the object’s pose. This allows us to travel back to the exact position where the robot sees the object, and then we can have the robot travel directly towards the object until we reach its actual position.
This poses another problem: some objects are not within the LiDAR’s range; that is, they may be smaller than the robot or above the robot. In this instance, it will not be possible to use sensor distance as a metric to determine how close the robot is to the object. This is why we must still use the depth estimation algorithm to determine an extremely rough guess of how close the robot is. While it is impossible to obtain an accurate depth reading from a monocular depth algorithm, it is still possible to get a relative reading, where closer objects will have a high value and distant objects will have a low value. Additionally, our object segmentation algorithm provides bounding boxes that indicate where the object is in our camera frame, which allows us to select just the object’s relative depth. In this way we are able to get closer to the object by getting the bounding box of the object, calculating the average of the object’s depth, travelling towards it, and then making sure our relative depth estimate isn’t too high. In the case that the object disappears from view for too long or the depth estimate goes above the threshold, we simply stop the robot because we assume either we are too close or the object is no longer in our view due to elevation or movement.

The object’s depth is calculated very simply: we take the median of all depths in the bounding box (the majority should obviously belong to the object) and filter out all depths less than the median. We are likely then left with all of the depths belonging to the object. We then take the mean of all those depths, and are left with our assumed relative distance of the object.
Sample Code:
```       box = self.predictions['bboxes'][object_prediction_index]

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
```def non_max_suppression(self, boxes, scores, threshold):
       order = sorted(range(len(scores)), key=lambda i: scores[i], reverse=True)
       keep = []
       while order:
           i = order.pop(0)
           keep.append(i)
           for j in order:
               # iou between the two boxes
               intersection = max(0, min(boxes[i][2], boxes[j][2]) - max(boxes[i][0], boxes[j][0])) * \
                           max(0, min(boxes[i][3], boxes[j][3]) - max(boxes[i][1], boxes[j][1]))
               union = (boxes[i][2] - boxes[i][0]) * (boxes[i][3] - boxes[i][1]) + \
                       (boxes[j][2] - boxes[j][0]) * (boxes[j][3] - boxes[j][1]) - intersection
               iou = intersection / union

               if iou > threshold:
                   order.remove(j)
       return keep
```





# Navigation
The navigation system described here is a modular framework that combines SLAM, path planning, exploration, and manual waypoint navigation. These components work in concert to allow a robot to autonomously explore an unknown environment, plan efficient paths, and adapt to changes in real-time. The framework is based on the ROS (Robot Operating System), which ensures flexibility and scalability.
Navigation in robotics requires precise localization, obstacle avoidance, and path planning. Implementing Simultaneous Localization and Mapping (SLAM) is critical because it enables the robot to build and maintain a map of an unknown environment while determining its position within that map. This capability forms the backbone of autonomous navigation, especially in dynamic or previously uncharted environments.
## SLAM gmapping

In environments where no pre-existing maps are available, the robot needs to simultaneously map the surroundings and localize itself.\
SLAM solves the "chicken and egg" problem: you need a map to localize and localization to create a map.\
SLAM allows for adaptability in dynamic environments where obstacles or features may change over time. 

Purpose: To create a map of the environment while simultaneously determining the robot’s location within it. \
Importance: Maps are essential for navigation in unknown environments. \
Localization is key to determining the robot's pose relative to obstacles and goals.

How it Works: \
The Gmapping node subscribes to sensor data (e.g., laser scans) and odometry data to update the map. \
`odom_frame`: Tracks the robot’s movement relative to the map. \
`map_update_interval`: Adjusts how often the map is updated, balancing accuracy and computation cost. 


##  Frontier Exploration

While SLAM enables mapping, the robot needs to autonomously decide where to explore next to cover the environment effectively.
Frontier exploration identifies regions that are on the boundary between explored and unexplored areas, allowing the robot to focus on expanding the map systematically.

Purpose: Autonomous exploration of an unknown environment. \
Importance: Automates navigation to unvisited areas by identifying frontiers which are boundaries between known and unknown regions. \
Saves manual effort and accelerates mapping of the environment. \
How it Works: \
The `explore_lite` node generates navigation goals to frontiers. \
These goals are sent to the `move_base` node for execution.



## Costmaps

Navigation requires both global and local planning to ensure safety and efficiency:
The global costmap is used for long-term planning and navigation across large areas.
The local costmap is used for real-time adjustments to avoid dynamic obstacles like people or moving objects.

Costmaps are grid-based representations of the environment, used for path planning and obstacle avoidance.

Global Costmap: \
Purpose: Provides a high-level view of the entire environment for long-distance path planning. \
global_frame: Uses the map frame to align with SLAM-generated maps. \
static_map: Indicates the use of a prebuilt map for planning. \
Importance: Enables efficient navigation to distant goals while avoiding large obstacles. 

Local Costmap: \
Purpose: Provides a localized, real-time view for immediate obstacle avoidance. \
rolling_window: Updates the local costmap as the robot moves. \
width, height: Define the area covered by the local costmap. \
Importance: Ensures the robot avoids unexpected obstacles in its immediate vicinity. 

Common Costmap Parameters: \
Define how obstacles are detected and inflated for safety: \
obstacle_range: Maximum detection range for obstacles. \
inflation_radius: Adds a buffer zone around obstacles for safety. \
robot_radius: defines how large the robot is so it knows which path is safe and where to make it more costly. 

## Path Planning 

Robots need to compute paths that are both efficient (shortest path) and safe (avoiding obstacles).
Integration of SLAM and costmaps allows the robot to dynamically adapt its path when new obstacles are detected.

Purpose: To compute and execute paths from the robot’s current position to a specified goal. \
Importance: \
Integrates global and local planning for robust navigation.  
Adapts dynamically to changes in the environment using costmaps. \
How it Works: \
Global planner for high-level paths. \
Local planner for immediate movements and obstacle avoidance. \
Configurations (base_local_planner_params.yaml): \
max_vel_x: 0.45 \
min_vel_x: 0.1 \
max_vel_theta: 1.0 \
acc_lim_theta: 3.2 \
acc_lim_x: 2.5 \
These ensure smooth and safe movement by limiting velocities and accelerations.

## Manual Waypoint Navigation
Purpose: Allows users to manually save poses and command the robot to navigate to those poses. \
Importance: \
Used for our gesture recognition to remember a specific pose when it recognizes an object and be able to navigate back to it. \
Specific gestures allow it to save the specific position and navigate back. 

# Usage

To run the robot, simply run roscam.py (type “python3 roscam.py” into your terminal/cli).
We only use an image subscriber and publisher for debugging.


# Story:
Neither Jeffrey nor I have ever been a pet owner of dogs and so we thought it would be really interesting if we were to create a robot that could follow gestures since it wouldn’t really be able to listen to our voices. 
The initial outline for the project was to use neural networks combined with google media pipe to recognize and train gestures and gesture recognition. Then, it would perform pre programmed commands such as to move around, spin, or to go back to an original position. However, it wouldn’t have the self navigating aspect that an actual dog would have. 
This gave us the idea to implement the GOAT (go to any thing) from class, because it was interesting and it would also be able to replicate fetching. This required us to have object recognition, depth recognition, and SLAM. After deciding on doing these implementations, we needed to do extensive research on what we could do to reduce overhead on the bot since if it was too demanding many errors could occur while running. 




## How it unfolded

Initially, we began by researching ways to implement gesture recognition and we tried to use different ways to send this information to the bot. There was a leap camera in the lab that we tackled but since it was purchased by a different company this version no longer worked. Then we decided to combine neural networks, open cv, and google media pipe which would create landmarks on our hands and then calculated the position of the landmarks relative to the wrist which would give us outputs of ids which mean certain gestures. 

Then, we branched off to work on different parts that, when it came together, would be able to replicate aspects of the GOAT system. 

Jeffrey worked on implementing a depth estimation algorithm to estimate depth based on images from the raspicam, as well as object segmentation to classify objects.

Leo worked on live mapping, localization, and path following so that the bot can see an object, save its frame as a pose and then be able to go back to it when it moves to another location. 

The project achieved its goals of gesture recognition and autonomous navigation to an object. We did not expect the cluster to be able to handle multiple heavy algorithms at the same time, so being able to successfully run object segmentation, depth estimation and gesture recognition at the same time on top of a ROS navigation stack is quite unexpected. Most of the time, a lot of issues occur when translating outputs between algorithms, so it was very pleasing to see that most of the data was actually extremely organized and very easy to both use and interpret. Because of this, the way that the algorithms interact is actually quite smooth. Overall, given our limitations and workarounds, we are happy to see that the robot works and functions as intended. 


