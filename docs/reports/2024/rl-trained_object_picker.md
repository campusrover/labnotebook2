---
title: Reinforcement Learning & Imitation Learning in Robotics
author: Sonny George & Alex Danilkovas
date: Dec 10 2024
---
# RL-Trained Object Picker

* Sonny George & Alex Danilkovas
* Dec 10 2024
* ROS version: _Noetic_

Please see [this youtube video](https://youtu.be/48MWSUeu9Rg) for a detailed walkthrough of the code.

## Problem Statement

Using Gazebo simulations, can we prototype an object-picking reinforcement learning loop for a simple task that generalizes to the real px100 robot arm?

## Motivation

The intersection of machine learning and robotic control is a notoriously difficult problem space. Unlike most machine learning problems that require one inference at a time, robotic control is a sequential decision-making problem where action taken not only influences the immediate outcomes, but also affects future states, creating a complex feedback loop. Furthermore, every action (control) needs to be a function of high-dimensional "real world" data sources that are often noisy, incomplete, and non-deterministic. Also, what characterizes a "good" action? Most machine learning applications are self-supervised and have data to learn from. However, what is the data in robotics?

### Reinforcement Learning & Imitation Learning

Much of the research in this are has focused on two paradigms: reinforcement learning (RL) and imitation learning (IL). In RL, the agent learns to interact with the environment by taking actions and observing the rewards it receives. In IL, the agent learns to mimic the behavior of an expert by observing the expert's actions. Often these methods are used in conjunction to allow a robot to learn from demonstration (IL), and then refine its policy using RL to learn how to make its policy robust to perturbations.

### So When is Reinforcment Learning _Actually_ Useful in Robotics?

For tasks like picking up objects that are always placed in the same location (what we are doing), reinforcement learning is **not** the most efficient way to solve the problem. In these cases, a rule-based system with something like PID control to ensure precise movements is recommended.

Reinforcement learning becomes useful when the task is complex and the environment is non-deterministic in such a way that a rule-based system would be too complex to design. However, the tradeoff is that RL requires a lot of data and time to train, and the policy learned may not be interpretable or generalizable to other tasks. Furthermore, (as we have learned in our project) it is not always guaranteed to work well within the time and compute restraints of the real-world.

## Relevant Literature

- For a primer on reinforcement and imitation learning in robotics, see [this FAQ](https://campusrover.github.io/labnotebook2/faq/ai/reinforcement_learning_%26_imitation_learning/).
- For a primer on the Python `gymnasium` package, see [this FAQ](https://campusrover.github.io/labnotebook2/faq/ai/rl_with_gymnasium/).
- For the original papers proposing the learning algorithms we use, see: [SAC](https://arxiv.org/abs/1801.01290), [PPO](https://arxiv.org/abs/1707.06347), [Behavioral Cloning](https://arxiv.org/abs/1805.01954).

## Codebase Details

For our project, we created a ROS package called `object-picker` with code that, for the Interbotix px100 robot arm:

1. Uses reinforcement learning (either PPO or SAC learning algorithms) to train a multi-layer perceptron policy to pick up a _'T'_-shaped object
2. Optionally, initialize the policy with imitation learning (behavioral cloning)
3. Runs trained policies on the real px100 robot arm
4. Contains pre-trained models (policies) from our training experiments

### Code Structure

Our repository is a `catkin` workspace containing a `src` directory with the `object-picker` package.

The `src` folder of this package (`object-picker`) is structured as follows:

```python
📁 src
├── 📁 models    # our trained models
├── 📄 config.py # global constants and parameters
├── 📄 env.py    # training-env code (reward logic & training primitives)
├── 📄 gazebo.py # code handling topic and service comms with gazebo sim
├── 📄 run.py    # loads and runs policies on real px100
├── 📄 train.py  # main entrypoint for training
└── 📄 utils.py  # generic helper functions
```

### How to Run the Code

ℹ️ **Run training:** To run the training script with the parameters specified in the `if __name__ == '__main__':` block, you must:

1. Launch ROS with:
    ```bash
    roslaunch
    ```
2. Start the Gazebo simulation with:
    ```bash
    roslaunch interbotix_xsarm_gazebo xsarm_gazebo.launch robot_model:=px100 use_position_controllers:=true gui:=false use_rviz:=true
    ```
    Or (depending on whether you want the Gazebo GUI or RViz to open)
    ```bash
    roslaunch interbotix_xsarm_gazebo xsarm_gazebo.launch robot_model:=px100 use_position_controllers:=true gui:=true
    ```
3. Start the training script with:
    ```bash
    rosrun object-picker train.py
    ```
    Or, `cd` into the `src` directory and run:
    ```bash
    python train.py
    ```


ℹ️ **Run real px100:** To run the trained models on the real px100 robot arm:

1. Run:
    ```bash
    rosrun object-picker run.py
    ```
    Or, `cd` into the `src` directory and run:
    ```bash
    python run.py
    ```

### Nodes Created

| Node Created | Function |
| --- | --- |
| `px100-training` | Orchestrate policy training by:<br>1. subscribing to the joint-state topics<br>2. publishing to joint-control topics |

### Topics Used

| Topics and Their Messages | Function |
| --- | --- |
| `/px100/waist_controller/state` | publish waist position |
| `/px100/waist_controller/command` | receive target waist position commands |
| `/px100/shoulder_controller/state` | publish shoulder position |
| `/px100/shoulder_controller/command` | receive target shoulder position commands |
| `/px100/elbow_controller/state` | publish elbow position |
| `/px100/elbow_controller/command` | receive target elbow position commands |
| `/px100/wrist_angle_controller/state` | publish wrist angle position |
| `/px100/wrist_angle_controller/command` | receive target wrist angle position commands |
| `/px100/right_finger_controller/state` | publish right finger position |
| `/px100/right_finger_controller/command` | receive target right finger position commands |
| `/px100/left_finger_controller/state` | publish right finger position |
| `/px100/left_finger_controller/command` | receive target right finger position commands |

## Technical Details & Discussion

### Observation & Action Space

Typically, a robot arm policy would observe:

1. Joint positions (sine and cosine of joint angles)
2. Joint velocities (joint angle derivatives) of all actuated joints
3. Joint torques from the last time step
4. Other task-specific signals

And output joint torques as actions.

Given our time constraints and levels of expertise, the only control interface we could reliably get to work for both the Gazebo-simulated px100 and real px100 was joint position control (and even then, the left finger didn't publish position state). That is, an API that allows for the setting and reading of joint positions.

To retrofit this higher level of abstraction to something more akin to what is typical for trained robot-arm policies, we wrote code that (1) takes a **_change_** in joint position (positive or negative value within some small range) and then, (2) by adding this change to the current joint positions, sets the new joint positions--making our control interface analogous to `cmd_vel`, whose input space is a target velocity (position over time).

Altogether, our simplified observation space was _only_ the joint positions, and our action space were these target changes in joint positions per step (a velocity).

Since the under-the-hood joint position controller uses PID, we had to consider the tradeoff between wait-time per step and movement precision. We could wait multiple seconds for every tiny step of movement--allowing the PID to steer the joint to the precise destinition--or, we could move quickly with less precision. Of course, it wouldn't make for much of a demo if the robot arm took 10 minutes to pick up an object, so we opted for low-precision movements. Of course, having a highly non-deterministic simulation--where controls are capable of producing high-variance outcomes--makes the training task much more difficult.

### Reward Function

Originally, our reward function included the following two terms:

1. **_'Assuming lifting position'_ term** The negative of the distance between the gripper and the object.
2. **_'Lifting the object'_ term:** The negative of the distance between the object and the goal position (raised in the air).

However, having little success at encouraging the arm to approach the object in a way that would permit it to be picked up, we changed the reward function to be stateful as follows:

1. **_'Assuming lifting position'_ term** term:
    - **_'Get low'_ term:** Z-value of gripper
    - **_'Get close'_ term:** Only once the gripper is low (and not before), the negative of the distance between the gripper and the object (since the forklift must get low before it can insert).
        - Once the gripper is both low and close (and not before), the _'get low'_ term is removed and replaced with a constant `1.0` (to prevent the _'get low'_ term from discouraging the lifting up of the object).
2. **_'Lifting the object'_ term:** (unchanged)

### Learning Algorithms

While we originall were using [SAC](https://arxiv.org/abs/1801.01290) (Soft Actor-Critic) for training, we switched to [PPO](https://arxiv.org/abs/1707.06347) (Proximal Policy Optimization) because we believed it could be more stable without tuning hyperparameters. The only hyperparameter we modified (from the defaults in the `stable-baselines3` library) was the number of steps per epoch (from 2048 to 4096).

Finally, we used [Behavioral Cloning](https://arxiv.org/abs/1805.01954) to initialize the policy by learning from single successful demonstration of the task. We hard-coded this demonstration (in the `config.py` file) and used it to train the policy for 100 epochs.

### Our Results

#### 1️⃣ Generation One (**SAC**)

For our first generation, we used SAC to train the policy. We our older reward function and ran it for 670,000 steps. Unfortunately, this policy, although it would often get close to the object, struggled to get underneath the object in a way that could pick it up. This motivated us to change our reward function as previously described.

![sac-670k gif](https://media.giphy.com/media/BHdB2I8dGUvZPkBwFI/giphy.gif)

#### 2️⃣ Generation Two (**PPO**)

For our second generation, we used PPO to train the policy. We used our new reward function and ran it for 270,000steps.

Here is what it looked like at the 130,000th step:

![ppo-130k gif](https://media.giphy.com/media/ybhp0Ibv2JXtQVDxWr/giphy.gif)

Obviously, it learned to "get low" (satisfying this aspect of the reward function). However, it is clearly struggling to approach the object once low.

After 270,000 steps, it is not much better (arguably worse):

![ppo-270k gif](https://media.giphy.com/media/o0SaSY4vefzhHIw7BN/giphy.gif)

#### 3️⃣ Generation Three (**PPO + Behavioral Cloning**)

Finally, once we introduce behavioral cloning, where we begin the learning process by training the policy on a single successful demonstration of the task before kicking off the PPO process, we see that a much more successful policy is achieved after only 40,000 PPO steps (approximately 1,400 episodes):

![ppo+bc-40k gif](https://media.giphy.com/media/b21InhRavoHLxejE6a/giphy.gif)

The following is a plot of the average reward per episode over the course of PPO training. As we can see, PPO is indeed increasing the average reward per episode over time.

![reward chart](https://i.imgur.com/xfc5s6Z.png)

Noteably, there is still a lot of variance, with a dense clusters around 0.0 and -0.5. There is a simple explanation to this. Because of the low-precision movements explained in [the observation and action space section](#observation--action-space), much of the time, these unavoidable imprecisions result in a light bump to the object that topple it over. When this occurs, the episode's average reward sufferes from never receiving reward for the lifting of the object toward the target position. Crucially however, we see that the PPO is gradually learning to output controls that minimize the chance of this occuring.

On the real robot, this learned policy does indeed pickup a _'T'_-shaped object. However, there definitely seem to be slight discrepancies between the simulation and the real world (notice the nose angle of the gripper).

![real px100 gif](https://media.giphy.com/media/NA0zhNFkZt9iVuRU2r/giphy.gif)

## Project Journey

The journey for this project was *laborious* and educational. The PX100 robotic arm is unlike any other robot in the lab—it’s immobile and plugged into a desktop computer. Naturally, the first step was to get the arm booted and working.  

When we powered up the computer, we were greeted by an Ubuntu 18.04 interface, without ROS installed. This immediately seemed off, as the arm had been used just last semester. Furthermore, the device wasn’t set up with internet connectivity.  

Fortunately, we resolved these issues within a couple of days and officially got started. Our first priority was to familiarize ourselves with the arm. To do this, we reviewed past projects and experimented with an API (`brl-pxh-api`) created by a previous student in the course.  

While the API was well-designed, we discovered it was incompatible with Gazebo. It functioned only on the physical robot, so we needed to dive deeper. 

Next, we researched which Reinforcement Learning (RL) algorithm would best suit our task. After some deliberation, we selected *Soft Actor-Critic (SAC)* as our starting point. We chose SAC for its **sample efficiency** and its ability to handle **sparse rewards** effectively. Around this time, we also explored how others had implemented RL with Gazebo. This led us to a small but promising repository, [ros-gazebo-gym](https://github.com/rickstaa/ros-gazebo-gym). Coincidentally, its sample project also used *SAC* as the model of choice, which reinforced our decision.  

Before diving too tackling the entire learning pipeline, we focused on getting the arm moving in simulation as expected. As explained further in explained in [the observation and action space section](#observation--action-space), we realized that we couldn’t control it the typical RL way—by outputting torque to each servo. Instead, we had to rely on setting angular positions for each servo. This led to the tradeoff between precision and speed, which we discussed in the same section.

Once we had control of the robot in simulation, we started by (1) digging our teeth into the learning pipeline--learning as much as we could from [ros-gazebo-gym](https://github.com/rickstaa/ros-gazebo-gym)--and (2) implementing our own solution. We started navigating through the aforementioned example and tracing how it ran. Although [ros-gazebo-gym](https://github.com/rickstaa/ros-gazebo-gym) contained far too many unnecessary and buggy features for our project, we were able to learn the basic abstraction associated with the (gymnasium)[https://github.com/Farama-Foundation/Gymnasium] framework. We came to know exactly what we needed in our solution and what we did not. After this, multiple 10+ hour days were spent writing out the training script—which rather than just a simple training script turned out to be an **end-to-end** training pipeline using Gazebo, Gymnasium, ROS, and Stable-Baselines3, with different modules of abstraction for modeling and manipulating the simulated environment, modeling and manipulating the PX100, as well as handling the training and reward of the task at hand.

Eventually we were able to get some sort of training running, and boy did it feel great seeing the robotic arm slowly make its way towards the red ball it wanted to pick up. However, our work was very far from done.

Next came optimization, tuning, and debugging—all of course paired with lots of experimentation. 

First matter of optimization was to increase the speed at which the simulation ran, because currently it ran at about 0.9 times real time speed. It would take far too long to train a model at this rate. Messing with the many Interbotix world-launch files, we were able to speed up the simulation to about 3x real-time speed. These speeds still weren't ideal, but still a great improvement. A little while later we were graciously granted access to a VNC much more powerful than the PC tower we were currently performing our training on. This not only helped us train at a rate of about 10x real time, but also allowed for us to test and run multiple training sessions at the same time.

In terms of tuning, we messed with how much each joint could move at maximum in a single step, as well as joint limits, and amount of steps per episode. One crucial finding was that our original budget of 15 steps per episode was too low for the robot to realistically pick up the object.

We adjusted and revamped the reward function many times. It started out as a simple linear 2 part reward, which then eventually transitioned into a non linear reward, which then after many more iterations became a reward with states. The results of this is described further in [the reward section](#reward-function). All of this was done to help guide the algorithm into succeeding at the task we had given it. 

However, despite all of that all it seemed the arm ever did was get close to the block/sphere, push it around sometimes maybe, or, after enough training, it would put its fingers around the block seemingly attempt to lift it but the block would stay on the ground. This baffled us and we eventually decided to test if it was actually possible to pick up the block in simulation—something we didn't even think to test beforehand (definitely an oversight on our part). 

Our findings were that while yes it was still possible, it was very difficult and there was very little room for error, way less than in real life. We found it to be an issue of pressure as well as the way the Gazebo physics engine modeled friction and other objects, which made it very difficult to do these kinds of things. There were a few things we could try, but time was starting to become a worry and we opted to change the object and remove the gripper friction issue altogether. We changed from a block to a T shape as something that the grippers would go under and scoop to pick up (like a forklift).

Finally, we started experimenting with other algorithms as well, such as Proximal Policy Optimization (PPO) as well behavior cloning in hopes of making it easier for it to find the movements which correlate with picking up the object. Luckily, we did experience observable and empiral positive results with the combination of PPO and behavioral cloning, as described in [the results section](#our-results).

Altogether, this project was profoundly challenging and deeply rewarding, requiring perseverance, creativity, and critical problem-solving. Navigating the labyrinth of technical hurdles, from setting up the hardware and software environment to designing a complex training pipeline, taught us valuable lessons in collaboration and problem solving. While the arm’s ultimate mastery of the task may have been tempered by the limitations of the simulation and control interface for the robot, the journey was a rich and rewarding experience that we will carry with us into future projects.
