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
In order for the robot to accurately be able to get from one pose to another with object avoidance, we needed to implement a form of SLAM (Simultaneous Localization and Mapping). This is because the robot needs to accurately move around, which requires a map. However, in order to build a map, the robot needs to move around. 

## Frontier search

In order for us to have a way to create a live map, each possible point that the robot can move to needs to be able to be kept track of live. So first we would find frontier cells in an occupancy grid which would represent where we have and haven’t explored. After we have attained the grid, we would use BFS to traverse the grid and collect a series of points to create a meaningful “frontier”. A frontier is a cell that represents the boundary between the known and unknown parts of the map and need to be identified. Each frontier would be filtered by their size, classified by the number of cells. The centroid for each frontier is then calculated by taking the average of all cell coordinates within the cluster which provides a goal point for the robot to navigate. 
```python
def search(
    mapdata: OccupancyGrid,
    start: "tuple[int, int]",
    include_frontier_cells: bool = False,
) -> "tuple[FrontierList, list[tuple[int, int]]]":
    MIN_FRONTIER_SIZE = 8  
    queue = [start]  
    visited = {start: True}  
    frontiers = []
    frontier_cells = []  

    while queue:
        current = queue.pop(0)
        for neighbor in PathPlanner.neighbors_of_4(mapdata, current):
            neighbor_value = PathPlanner.get_cell_value(mapdata, neighbor)
            if neighbor_value >= 0 and neighbor not in visited:
                visited[neighbor] = True
                queue.append(neighbor)
            elif FrontierSearch.is_new_frontier_cell(mapdata, neighbor, is_frontier):
                is_frontier[neighbor] = True
                new_frontier, new_frontier_cells = (
                    FrontierSearch.build_new_frontier(
                        mapdata, neighbor, is_frontier, include_frontier_cells
                    )
                )
                if new_frontier.size >= MIN_FRONTIER_SIZE:
                    frontiers.append(new_frontier)
                    if include_frontier_cells:
                        frontier_cells.extend(new_frontier_cells)
    return (FrontierList(frontiers=frontiers), frontier_cells)
```
Starting from the robot’s current pose, it would perform bfs to all mappable cells and identify frontiers by checking unknown cells that have a free neighbor.

For frontier search, you would need to pass an occupancy grid to the search method along with the robot's starting grid coordinates and then the class would return a list of frontiers which contains the number of cells and the centroid which is the goal for navigation. 

## Path Planner
With all these points, you need to plan the way to get to the goal pose. This class would implement the path planning using an A* algorithm to compute optimal paths between two points as well as generating cost maps. 
```python
@staticmethod
def a_star(mapdata: OccupancyGrid, cost_map: np.ndarray, start: "tuple[int, int]", goal: "tuple[int, int]"):
    COST_MAP_WEIGHT = 1000
    if not PathPlanner.is_cell_walkable(mapdata, start):
        start = PathPlanner.get_first_walkable_neighbor(mapdata, start)
    if not PathPlanner.is_cell_walkable(mapdata, goal):
        goal = PathPlanner.get_first_walkable_neighbor(mapdata, goal)

    pq = PriorityQueue()
    pq.put(start, 0)
    cost_so_far = {start: 0}
    came_from = {start: None}

    while not pq.empty():
        current = pq.get()
        if current == goal:
            break
        for neighbor, distance in PathPlanner.neighbors_and_distances_of_8(mapdata, current):
            new_cost = cost_so_far[current] + distance + COST_MAP_WEIGHT * PathPlanner.get_cost_map_value(cost_map, neighbor)
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + PathPlanner.euclidean_distance(neighbor, goal)
                pq.put(neighbor, priority)
                came_from[neighbor] = current

    path = []
    cell = goal
    while cell:
        path.insert(0, cell)
        cell = came_from.get(cell)
    return path

```
This algorithm combines the cost to get to where the node currently is and also the estimated cost to the goal. Essentially, this means it gets the sum of distances travelled along with the euclidean distance to the goal. In order to avoid obstacles, the cost map would inflate the cost of them to make sure that the path would be a safe distance away from them. 

Use grid_to_world and world_to_grid to convert between map coordinates and robot frame.
Call a_star with start and goal coordinates to compute the optimal path.
Use calc_cost_map to preprocess the occupancy grid before planning.

## Pure Pursuit

Now that you have the path and be able to calculate the costs to go to the goal point, how would we be able to successfully move there? What pure pursuit does is that it takes the planned path from frontier exploration and converts it to velocity commands for the robot. This ensures smooth path following, real time adaptation, and obstacle avoidance. 

```python
def find_lookahead(self, nearest_waypoint_index, lookahead_distance) -> Point:
    i = nearest_waypoint_index
    while (
        i < len(self.path.poses)
        and self.get_distance_to_waypoint_index(i) < lookahead_distance
    ):
        i += 1
    return self.path.poses[i - 1].pose.position

```
This algorithm uses a point ahead of the path that the robot aims for which ensures smooth trajectory tracking. If it is too close the changes would be too abrupt and if it is too far the path could possibly be cutting corners or may stray from the actual path. 

```python
steering_adjustment = self.calculate_steering_adjustment()
turn_speed = self.TURN_SPEED_KP * drive_speed / radius_of_curvature
turn_speed += steering_adjustment
```
This would adjust the bot’s heading based on the lookahead point. 

```python

for dx in range(-self.FOV_DISTANCE, self.FOV_DISTANCE + 1):
    for dy in range(-self.FOV_DISTANCE, self.FOV_DISTANCE + 1):
        cell = (robot_cell[0] + dx, robot_cell[1] + dy)
        distance = PathPlanner.euclidean_distance(robot_cell, cell)
        if not PathPlanner.is_cell_walkable(self.map, cell):
            # Adjust steering based on proximity to obstacles

```
This uses a field of view approach to make the bot avoid walls or obstacles. 

The best way is to experiment with Lookahead_distance and obstacle_avoidance_gain values for optimal performance. 


## Frontier Exploration

In order to coordinate the frontiers that are being generated as well as the path planning algorithm, frontier exploration is essential to have autonomic exploration in an unknown environment. It serves to coordinate between frontier detection, path planning, and navigation execution. What it does is that it would explore frontiers which are unexplored regions, then it would compute the optimal paths to reach them, and then command the robot to go there. 

```python
def find_next_frontier(self, map_data: OccupancyGrid):
    start = PathPlanner.world_to_grid(map_data, self.pose.position)
    frontier_list, _ = FrontierSearch.search(map_data, start, self.is_in_debug_mode)
    if frontier_list is None or not frontier_list.frontiers:
        rospy.logwarn("No frontiers found.")
        return None
    largest_frontier = max(frontier_list.frontiers, key=lambda frontier: frontier.size)
    return largest_frontier.centroid

```
This uses the FrontierSearch class to identify the most promising frontier and then converts the frontier centroid to a goal position in the occupancy grid.


Afterwards, it uses pathplanner to compute an optimal path to the selected frontier and uses a cost map to account for obstacles. When all frontiers are explored, it would stop exploring. 

The robot should be publishing to /map and /odom with frontier search and path planner implemented and imported and be able to publish the path for pure pursuit. 

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


