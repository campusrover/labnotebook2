---
title: Calculating coordinates using AMCL and LiDAR
author: Chloe Wahl-Dassule
date: Dec 9 2024
---

## Author

* Chloe Wahl-Dassule

* Dec 9 2024

* ROS1

## Summary

This is a tutorial for calculating the coordinates of an object, point_a, detected on LiDAR, assuming AMCL is running.

## Details

### First get the current position of the robot using data from amcl. 
*Example:*


    def get_robot_position(self):
        try:
            self.tf_listener.waitForTransform('/map', '/base_link', rospy.Time(0), rospy.Duration(4.0))
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            
            x, y, z = trans  #Position in 3D space
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)
            
            return x, y, yaw  #Return position and yaw angle
            except (tf.Exception, tf.ConnectivityException, tf.LookupException) as e:
                rospy.logerr(f"TF Error: {e}")
                return None, None, None
            
	

### Take into account that the robot may not be facing a yaw of 0
Take the yaw calculation from step 1 and combine it with the LiDAR angle to figure out what angle the object is at from the origin of the map.

*Example:*
	
    def get_angle(self, yaw, lidar):
        lidar = lidar
        if lidar > 180:
            remainder = lidar - 180
            lidar = (180 - remainder) * -1
            result = math.radians(lidar) + yaw
            return result
        
### Simple trigonometry to get the coordinates of point_a, based on the distance reading and the angle from step 2
Make sure to convert degrees to radians

*Example:*

    def get_coord(self, distance, radians, coordinate):
        x = coordinate[0]
        y = coordinate[1]
        new_x = x + distance * math.cos(radians)
        new_y = y + distance * math.sin(radians)
        return new_x, new_y
