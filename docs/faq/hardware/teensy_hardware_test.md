---
title: Teensy Hardware
desc: How to test the  hardware
date: 29-jun-2024
status: new
author: pito
---
# Teensy Hardware Test

All of our homemade robots are based on the [Linorobot](https://github.com/linorobot/linorobot/wiki/1.-Getting-Started) code base. They define specifically what motors, motor drivers, imus, and lidars are supported. Stick to those exact ones. Don't get clever and assume that another model looks the same so it must be compatible!

We always use the [Teensy Microcontroller](https://www.pjrc.com/teensy/). When building a new one it is highly desireable to test it out without ROS with only the teensy.

## Setup

The teensy has a usb port that normally is connected to the Rasberry Pi on the robot. To do the testing you will connect the teensy to your mac or pc.

### Software Setup

Use the [Teensyduino](https://www.pjrc.com/teensy/teensyduino.html) software package to run the Arduino IDE with Teensy specific configuration. Install the software on your computer, connect the USB, and verify that you have a good set up by installing and running the simplest code, which is the famous blink sketch. Just follow [these instructions.](https://www.pjrc.com/teensy/tutorial.html)

### What to test

You want to verify:

1. The motors are responding correctly. In our software, MOTOR1 is on the let ("driver side") and MOTOR2 is on the right ("passenger side."). Test left and right for forward and backward at different speeds, as well as stopped. Forward motion of the motor is when the robot, on a surface would drive forward (this may be obvious)
2. Encoders are responding correctly. Disconnect power from the motors and manually turn them and see that the encoders are counting up and down based on the direction of motion.
3. i2c is connected correctly. Run the i2c detection script and make sure that it detects the right number of i2c devices. All we can do is check detection. Correctness has to wait for actually running ROS.

### Test Scripts

Under development! [Teensy Scripts to test Branbot hardware](https://github.com/campusrover/teensytest)
### Pins

You will need to have the right pin numbers to run the tests. As of now, here are the pins.

```
#define MOTOR1_ENCODER_A 14
#define MOTOR1_ENCODER_B 15
#define MOTOR2_ENCODER_A 12
#define MOTOR2_ENCODER_B 11

#define MOTOR_DRIVER L298
#define MOTOR1_IN_A 20
#define MOTOR1_IN_B 1
#define MOTOR1_PWM 22
#define MOTOR2_IN_A 6
#define MOTOR2_IN_B 8
#define MOTOR2_PWM 5
```
