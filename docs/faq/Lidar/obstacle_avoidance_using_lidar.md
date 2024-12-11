---
title: Obstacle Avoidance using LIDAR
author: Parthiv Ganguly
date: Dec 10 2024
---
# Obstacle Avoidance using LIDAR
## Author: Parthiv Ganguly

There are many scenarios where a robot has to navigate an environment that is not previously mapped. While SLAM can technically map the environment as the robot moves, in reality, this process is unreliable if the environment is completely unknown to the robot. In a case like this, straightforward obstacle avoidance algorithms can be helpful. This page details one that was helpful in our situation. This algorithm divides the 360 degrees around it into regions, and when faced with an obstacle in front, it will turn (by rotating backwards) to point to the most "obstacle-free" region which is closest (i.e. lowest angular difference) to the front region.

## Initialization

First, we define the variables referenced throughout the algorithm, in the `__init__` method of the class.
```Python
# If an object is detected in front of robot, "obstacle_detected" is set to True,
# and an avoid_angular_vel is calculated to avoid the obstacle
self.robot_state = {"obstacle_detected": False, "avoid_angular_vel": 0}
# div_distance keeps track of the LIDAR distances in each region,
# 0 is the front region, 1 is front-left, 2 is left, etc.
self.div_distance = {"0": [], "1": [], "2": [], "3": [], "4": [], "5": [], "6": [], "7": []}
# div_cost calculates the cost of region based on how far it is from 0, and the sign gives the direction
self.div_cost = {"0": 0, "1": 1, "2": 2, "3": 3, "4": 4, "5": -3, "6": -2, "7": -1}
```

## Using scan_cb to populate div_distance

scan_cb is the callback function to the /scan Subscriber, and is used to populate div_distance. Obstacles or LIDAR readings closer than `OBJ_THRESHOLD` are stored in div_distance and used to update the robot state and decide whether the robot is facing an obstacle. Adjust `OBJ_THRESHOLD` based on how much distance you want your robot to maintain from the obstacles.

```Python
def scan_cb(self, msg):
    for key in self.div_distance.keys():
        values = []
        if key == "0":
            # The front region is wider compared to the other regions (60 vs 45),
            # because we need to avoid obstacles in the front
            for x in msg.ranges[int((330/360) * len(msg.ranges)):] + msg.ranges[:int((30/360) * len(msg.ranges))]:
                if x <= OBJ_THRESHOLD and not(math.isinf(x)) and not(math.isnan(x)) and x > msg.range_min:
                    values.append(x)
        else:
            for x in msg.ranges[int((23/360) * len(msg.ranges)) + int((ANGLE_THRESHOLD/360) * len(msg.ranges)) * (int(key)-1) : int((23/360) * len(msg.ranges)) + int((ANGLE_THRESHOLD/360) * len(msg.ranges)) * int(key)]:
                if x <= OBJ_THRESHOLD and not(math.isinf(x)) and not(math.isnan(x)) and x > msg.range_min:
                    values.append(x)
        self.div_distance[key] = values
```

## Calculating robot_state

This function calculates whether or not an obstacle is in front of the robot (`self.robot_state["obstacle_detected"]`), and it calculates the angular velocity needed to avoid the obstacle and point in the most "obstacle-free" zone (i.e. zone that is without obstacles or has the farthest LIDAR reading) with the lowest cost (the closer the zone is to the front, the lower the cost).

```Python
def calc_robot_state(self):
    nearest = math.inf
    region_diff = 0
    # Regional differences are calculated relative to the front region
    goal = "0"
    # The 4th region gives the highest regional diff so we start with that
    max_destination = "4"
    max_distance = 0

    for key, value in self.div_distance.items():
        region_diff = abs(self.div_cost[key] - self.div_cost[goal])
        
        # If there're no obstacles in that region
        if not len(value):
            # Find the obstacle-free region closest to the front
            if (region_diff < nearest):
                nearest = region_diff
                max_distance = OBJ_THRESHOLD
                max_destination = key
        # Check if the region is the most "obstacle-free", i.e. the LIDAR distance is the highest
        elif max(value) > max_distance:
            max_distance = max(value)
            max_destination = key

    # Difference between the most obstacle-free region and the front
    region_diff = self.div_cost[max_destination] - self.div_cost[goal]

    # If the obstacle free path closest to the front is not the front (i.e. nearest != 0),
    # this means that there is an obstacle in the front
    self.robot_state["obstacle_detected"] = (nearest != 0)
    # The avoid_angular_vel is 0.7, and it's sign is the same as the sign of the regional difference
    # We do the max(1, ) thing to avoid division by 0 when the regional difference is 0
    self.robot_state["avoid_angular_vel"] = ((region_diff/max(1, abs(region_diff))) * 0.7)
```

## Bringing it all together

Have the following snippet of code inside your main ros loop, so that the robot state is constantly up to date. Replace the variables `linear_velocity` and `angular_velocity` with whatever linear and angular velocity the robot should have when the course is clear and it is heading directly towards its goal.

```Python
cmd_vel = Twist()
self.calc_robot_state()
if self.robot_state["obstacle_detected"]:
    # Avoid obstacle by turning back
    cmd_vel.linear.x = -0.1
    cmd_vel.angular.z = self.robot_state["avoid_angular_vel"]
else:
    cmd_vel.linear.x = linear_velocity
    cmd_vel.angular.z = angular_velocity
self.vel_pub.publish(cmd_vel)
```