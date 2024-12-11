# Line Follower Advanced  

**Author:** Eric Hurchey  
**Date:** Dec 2024  

## Summary  

This project involved building a Line Follower robot that could follow part of a line, stop at a specific destination, and transition to the next task.  

## Details  

### `image_cb` - Line Detection  
The `image_cb` function processes camera input to detect the line. It:  
1. Converts the image to the HSV color space to isolate a specific color range (orange).  
2. Masks the lower part of the image to focus on the area where the line is most likely visible.  
3. Identifies the largest contour of the line and calculates the error between the line's center and the image's center.  
4. Passes the error to a PID controller for movement adjustments.  

# Example of error calculation
cx = int(M['m10'] / M['m00'])  # Center x-coordinate of the line
err = cx - w // 2  # Error relative to the image center

### Movement Control  

The movement function uses the error value to adjust the robot's linear and angular velocities. A PID controller ensures the robot stays aligned with the line:  

# PID controller example
self.control_signal = (KP * err) + (KD * derivative)
self.twist.linear.x = MAX_SPEED
self.twist.angular.z = -self.control_signal

### Behavior  

- **Line Following**: The robot follows the detected line, continuously adjusting its path.  
- **Stopping at Destination**: Once the robot reaches a predefined destination, it stops following the line.  
- **Task Transition**: After stopping, the robot transitions to the next task, such as approaching a target or turning around.  

### Key Features  

- Uses OpenCV for image processing and line detection.  
- PID control for smooth and responsive movement.  
- State-based logic for task transitions.  

### Improvements  

- Fine-tuning the mask area to handle different lighting and line conditions.  
- Enhancing PID parameters to reduce overshooting or oscillations.  
- Adding robustness for handling line gaps or sharp turns.  
