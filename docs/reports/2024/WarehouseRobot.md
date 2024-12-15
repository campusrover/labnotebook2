# Warehouse Robot Navigation

This project implements an autonomous robot system for warehouse navigation using ROS (Robot Operating System). The robot is capable of detecting and following a colored line, identifying fiducial markers, and returning to its starting position. It uses state-driven logic, computer vision, and odometry to achieve its objectives.

## Features

- **Line Following**: Detect and follow an orange line using a camera.
- **Fiducial Detection**: Identify fiducial markers and approach them precisely.
- **State-Based Navigation**: Transition through multiple predefined states for flexible navigation.
- **Return to Home**: Navigate back to the starting position after completing tasks.
- **Dynamic Speed Control**: Adjust speed dynamically based on proximity to fiducials and line positions.
- **User Interaction**: Receive user input for mission control and state transitions.

---

## Technical Description

### High-Level Design

The robot operates in multiple predefined states, managed by the `calculate_movement` function:
- **LINE_FOLLOWING**: Follow the orange line.
- **APPROACHING**: Move toward a fiducial marker.
- **RETURN_TO_LINE**: Navigate back to a stored line position after fiducial interaction.
- **TURN_TO_HOME**: Calculate and turn toward the starting position.
- **RETURNING**: Follow the line back to the home position.
- **TURN_180**: Perform a 180-degree turn to prepare for the next fiducial.

State transitions are based on sensor inputs and environmental conditions.

---

### Key Functional Modules

1. **Image Processing for Line Following**:
   ```python
   hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
   mask = cv2.inRange(hsv, LOWER_ORANGE, UPPER_ORANGE)
   M = cv2.moments(largest_contour)
   cx = int(M['m10'] / M['m00'])
   err = cx - w // 2
   ```

2. **PID Control for Line Following**:
   ```python
   derivative = (err - self.prev_err) / delta_time
   self.control_signal = (KP * err) + (KD * derivative)
   self.twist.angular.z = -self.control_signal
   ```

3. **Fiducial Detection and Localization**:
   ```python
   transform = self.tf_buffer.lookup_transform('base_link', f'pin_{self.target_fiducial}', rospy.Time(0))
   self.pin_position = transform.transform.translation
   self.distance_to_pin = math.sqrt(self.pin_position.x**2 + self.pin_position.y**2)
   self.angle_to_pin = math.atan2(self.pin_position.y, self.pin_position.x)
   ```

4. **State-Based Movement**:
   ```python
   dx = self.temp_line_position[0] - pos.x
   dy = self.temp_line_position[1] - pos.y
   dist_to_temp = math.sqrt(dx*dx + dy*dy)
   self.twist.linear.x = min(MAX_SPEED, dist_to_temp * 0.5)
   ```

5. **Return to Home**:
   ```python
   target_yaw = math.atan2(dy, dx)
   yaw_diff = self.normalize_angle(target_yaw - self.current_yaw)
   self.twist.angular.z = turn_speed if yaw_diff > 0 else -turn_speed
   ```

## Error Handling

### Fiducial Loss
```python
rospy.logwarn("Lost sight of fiducial during approach")
```

### Odometry Failures
```python
try:
    odom = rospy.wait_for_message('/odom', Odometry, timeout=1.0)
except rospy.ROSException:
    rospy.logwarn("Failed to get current position")
```

## Usage Instructions

### Prerequisites
- Install ROS and ensure the required topics (/cmd_vel, /odom, /raspicam_node/image/compressed) are available.
- Ensure the robot's environment has an orange line and fiducial markers (IDs: 106, 100, 108).

### Running the Robot
1. Launch ROS and required nodes.
2. Run the script:
   ```bash
   rosrun <package_name> warehouse_robot.py
   ```
3. Input the target fiducial ID when prompted:
   ```plaintext
   Enter the target fiducial ID: 106
   ```

### User Interaction
During fiducial interaction, you can:
- Enter `continue` to proceed to the next task.
- Enter `shutdown` to stop the robot.

## Workflow

### Initialization:
- The robot stores its starting position and orientation from odometry.
- Awaits user input for the target fiducial ID.

### Line Following:
- Uses the camera to detect and follow the orange line.

### Fiducial Interaction:
- Detects and approaches the fiducial marker.
- Pauses for user input to determine the next action.

### Return to Line:
- Returns to the stored line position after interacting with the fiducial.

### Return to Home:
- Follows the line back to the starting position.

### Turn 180 Degrees:
- Performs a 180-degree turn to align with the next fiducial or complete operations.

## Project Journey

### Development Evolution
The project underwent several iterations before arriving at its current form. Initially, the scope was different, but through various pivots and refinements, it evolved into a focused warehouse navigation solution. The most significant change came from reconsidering the initial project direction multiple times to find the most practical and achievable approach.

### Technical Challenges & Solutions

#### Key Challenges Overcome:
1. **Home Position Tracking**: A critical breakthrough came in solving the issue of saving and utilizing the home yaw orientation. This was essential for accurate return navigation and required careful consideration of coordinate systems and orientation tracking.

2. **Fiducial Approach Logic**: One of the more complex challenges was developing the logic for the robot to properly detach from the line when approaching fiducials. This required precise distance calculations and smooth transition states.

3. **Integration Complexities**: While the original plan included integration with Rafael, technical constraints led to focusing solely on Roba for execution. This taught valuable lessons about scope management and adapting to technical limitations.

### Development Process

The project's success relied heavily on:
- Extensive testing cycles with multiple test cases
- Dedicated debugging sessions, often spanning many hours
- Iterative code refinement through hands-on testing
- Continuous documentation and code review

### Lessons Learned

1. **Scope Management**: While the initial vision was more extensive, learning to adapt and focus on core functionality proved crucial for delivering a working solution.

2. **Testing Importance**: The numerous hours spent on debugging and testing highlighted the importance of thorough testing methodologies in robotics development.

3. **Future Improvements**: Areas identified for potential enhancement include:
   - Integration with additional robot platforms
   - More sophisticated path planning algorithms
   - Enhanced error recovery mechanisms

### Time Investment
The project required significant time investment, particularly in:
- Debugging and testing sessions
- Implementing and refining core functionalities
- Documentation and code organization
- System integration testing

This extensive testing and debugging phase, while time-consuming, was crucial for ensuring reliable robot behavior and robust navigation capabilities.

## Contributors
Eric Hurchey (erichurchey@brandeis.edu)
