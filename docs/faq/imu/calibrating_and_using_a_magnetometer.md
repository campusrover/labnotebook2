---
title: Calibrating and Using a Magnetometer
author: Artem Lavrov
date: Dec 9 2024
---
## Author
* Artem Lavrov
* Dec 9 2024
* ROS 1 (noetic)

## Why is this in the imu folder?

This is in the imu folder because the idea that the imu is one sensor is a lie. An imu usually consists of 3 separate sensors, each of which do different things. Two of these sensors are relative sensors (they give data relative to the robot's state when it was started - it's position and orientation - and this data may not necessarily be consistent between) and one of them is absolute (it gives its measurement based on the robot's position and orientation relative to the earth). The two relative sensors are the gyroscope and accelerometer. We will not be focusing on those two sensors in this article. Our focus is on the absolute sensor: the magnetometer.

## What is a Magnetometer

The magnetometer is essentially an electronic compass. It detects the magnetic field of the earth and publishes that data. More specifically, it measures the strength of the magnetic field along each of its axes. That's it. It seems very incredibly simple, yet it is deceptively tricky. The reason for this is because the earth is not the only thing that has a magnetic field. Most electronic components have magnetic fields, and also magnetic fields can be warped by materials, for example steel and a lot of other metals. If you've been paying attention you can see the major issue here. These are all abundant on a robot, so most likely, there will be a lot of interference with the magnetometer. To account for this interference, you need to calibrate the magnetometer, which I will teach you how to do in this article.

## Types of Interference

This isn't super important to understand unless you are making your own calibration script, but there are two types of interference. Hard iron interference is pretty straight forward, it comes from other magnets and results in your data being offset by a specific amount due to the pull of that magnetic field. Soft iron interference is a little more tricky: it comes from matetials (mostly metals) that warp magnetic fields, but do not generate a magnetic field. This can still be calibrated for, but requires a lot more math that I will not be explaining in this article.

## Gathering Data

In order to calibrate your magnetometer, you first want to collect a bunch of readings in every direction. I would do this by creating a script to log these values into a csv file. I mean literally just pull the x,y, and z values and write them to a csv. Here's a quick example of how to do just that.

Copy the python script below:
```python
#!/usr/bin/env python
import rospy
from sensor_msgs.msg import MagneticField
from lino_msgs.msg import Imu
import math
import matplotlib as mpl


class MagneticComparer():

    def magnetic_callback(self, msg):
        self.mag_x = msg.magnetic_field.x
        self.mag_y = msg.magnetic_field.y
        self.mag_z = msg.magnetic_field.z

    def __init__(self):
        rospy.init_node('magnetic_orientation', anonymous=True)
        self.mag_sub = rospy.Subscriber(
            "/imu/mag", MagneticField, self.magnetic_callback)
        self.imu_sub = rospy.Subscriber("/raw_imu", Imu, self.imu_cb)

        self.mag_x = 0
        self.mag_y = 0
        self.mag_z = 0

    def run(self):
        print(f"x,y,z")

        while rospy.is_shutdown is False and self.mag_x == 0:
            continue
        while rospy.is_shutdown() is False:
            print(f"{self.mag_x}, {self.mag_y}, {self.mag_z}")
            rospy.Rate(30).sleep()

if __name__ == '__main__':
    try:
        MagneticComparer().run()
    except rospy.ROSInterruptException:
        pass
```

and then dumping the output into a csv file like so
```bash
python3 mag_logger.py > magnetometer.csv
```

Of course, you can do this any way you like.

## Getting the Calibration Parameters

I do not recommend trying to figure out how to account for the two different types of interference yourself, but it is handy to know how to recognize the different types of interference.

Perfect data should look roughly like a circle around the origin. This would indicate no interference.

Hard iron interference will cause your data to be offset and not be centered around the origin.

Soft iron interference will cause your data to not look like a circle. It could make it more ellipsoid, or if it is really bad, it could just make it look completely random.

I would use someone else's script to calibrate these values, as the math for soft iron interference can be difficult. However it is not impossible. All you really have to do is fit the data to a sphere, find out the center of the sphere, and figure out the offset of the center of this sphere from the origin. If you don't want to do this yourself [here is a nice script that does this for you](https://github.com/italocjs/magnetometer_calibration).

## Using the Calibration Parameters
Assuming you're using the script linked to above, correcting the raw mag data once you have the correct calibration parameters should be pretty straightforward. First add or subtract your hard iron offsets to make sure your data is centered around the origin. Then use your soft iron values to fit your data to a circle. Here is an example based on the parameters of the script above (this assumes you are using ros):
```python
#! /usr/bin/python3
import numpy as np
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import Vector3
import rospy
from collections import deque

'''Calibration Params'''
# Replace with the offset values you get from calibration_plotter.py
OFFSETS = [-13, -33, -33]
# Replace with the scale values you get from calibration_plotter.py
SCALES = [1.083333, 0.962963, 0.962963]

''' Median Filter Params '''
# Adjust this to increase the buffer size for the median filter.
# Increasing will reduce noise, decreasing will increase noise.
WINDOW_SIZE = 20

# Create buffers for median filter
x_buffer = deque(maxlen=WINDOW_SIZE)
y_buffer = deque(maxlen=WINDOW_SIZE)
z_buffer = deque(maxlen=WINDOW_SIZE)


def apply_calibration(raw_mag_data, offsets, scales):
    '''
        Apply soft and hard iron correction to the raw data and return the 
        corrected data
    '''
    # Apply hard-iron correction
    corrected_data = np.array(raw_mag_data) - np.array(offsets)

    # Apply soft-iron correction
    corrected_data = corrected_data / np.array(scales)

    return corrected_data


def mag_callback(msg):
    '''
        ROS Subscriber callback to apply calibration and publish corrected data
    '''
    # Extract magnetometer data from the message
    raw_mag_data = np.array(
        [msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z])

    corrected_mag_data = apply_calibration(raw_mag_data, OFFSETS, SCALES)

    x_buffer.append(corrected_mag_data[0])
    y_buffer.append(corrected_mag_data[1])
    z_buffer.append(corrected_mag_data[2])

    filtered_data = (np.median(x_buffer), np.median(
        y_buffer), np.median(z_buffer))

    # Create a new message to publish the corrected data
    corrected_mag_msg = MagneticField()
    corrected_mag_msg.magnetic_field = Vector3(*filtered_data)
    corrected_mag_msg.header = msg.header

    # Publish the corrected data
    mag_pub.publish(corrected_mag_msg)


# Initialize ROS node
rospy.init_node('magnetometer_calibration')

# Create a publisher for the corrected magnetometer data
mag_pub = rospy.Publisher('/imu/mag_corrected', MagneticField, queue_size=10)

# Subscribe to the original magnetometer topic
rospy.Subscriber('/imu/mag', MagneticField, mag_callback)

# Spin to keep the node running
rospy.spin()
```
 
Now you should be publishing corrected mag data! Congrats! You can fuse this wiht your other imu data or use it to calculate your absolute heading (if you do go this route, make sure to account for magnetic declination). Thank you for reading and I hope this was useful to you in some way.
