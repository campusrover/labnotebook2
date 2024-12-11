---
title: General Impl./Workflow for RL with OpenAI Gymnasium, Gazebo, ROS, and RViz
author: Alex Danilkovas
date: Dec 10 2024
---

By Alex Danilkovas | adanilkovas@gmail.com, Dec 10 2024


This will be a general overview of how to implement your own Reinforcement Learning (RL) algorithm with the help of Gymnasium and a model of your choice. Before continuing, read the "Reinforcement Learning & Imitation Learning in Robotics" FAQ by Sonny if you are not yet familiar with RL.

## Getting Started
Before starting any RL project, you must consider the following:
1. What task is my robot going to perform?
2. How can I tailor a reward function to reward the robot for performing the task correctly?
3. What RL algorithm is best suited for this problem?

### In Deciding a Task
The task you choose for your robot defines every aspect of the RL pipeline. Consider tasks that are:
* **Specific**: Clearly defined objectives make it easier to design the reward function.
* **Feasible**: Ensure that the task is within the capabilities of the simulated model and the RL algorithm.
* **Transferable**: If deploying in the real world, choose tasks that closely match scenarios the robot will encounter.

For example:
* A wheeled robot navigating through a maze.
* A robotic arm placing objects into bins.
* A drone stabilizing its flight in windy conditions.

### Choosing the RL Algorithm
The algorithm depends on the complexity and nature of your task:
* **Discrete Action Space:** Use Q-learning, Deep Q-Networks (DQN), or variants like Double DQN.
* **Continuous Action Space:** Algorithms like Proximal Policy Optimization (PPO), Soft Actor-Critic (SAC), or Deep Deterministic Policy Gradient (DDPG) work well.
* **Multi-Agent Systems:** Explore Multi-Agent Reinforcement Learning (MARL) frameworks.

In all likelyhood you are going to be dealing with a continious action space, and therefore recomend I recomend looking into algorithms such as **Soft Actor-Critic** first. 

---

## Training Loop Overview
The training loop is the heart of the RL implementation:
1. Initialize the Gym environment and agent.
2. Start a training episode:
   * **Action:** The agent selects an action based on its policy.
   * **Simulation Step:** Send the action to the environment via ROS/Gazebo.
   * **Observe:** Receive the new state, reward, and done flag.
   * **Learn:** Update the policy using the RL algorithm.
   * **Reset:** Obtain the initial state from the environment for the next episode **after** the number maximum number of steps for an episode has been reached (or an exit condition has been met).
3. Save progress weights periodically with checkpoints through a `checkpoint()` callback.

### Reward Function Design
The reward function is critical as it directly influences the agent's learning process. Some principles to follow:
1. **Simplicity:** The reward function should be straightforward and closely aligned with the task objective.
2. **Clarity:** Avoid ambiguity in rewards; positive rewards should correspond to progress, and penalties should signal undesirable actions.
3. **Scaling:** Normalize rewards to keep them within a range, ie 0 and 1.

Example reward functions:
* **Navigation Task:** Positive reward for moving closer to the goal; penalty for collisions.
* **Grasping Task:** Positive reward for successfully grasping an object; penalty for dropping it.

Consider the whether you want a linear or curved reward function, it is quite helpful to plot functions in Desmos and then transfer them into your training script.

Below is a snipit of our *Pick and Place* reward function:
```
python

grabber_obj_dist = np.mean(
    [
        grabber_base_dist,
        grabber_base_dist,  # Weight the base distance more
        left_finger_dist,
        right_finger_dist,
    ],
    axis=0,
)
clipped_grabber_obj_dist = clip_value(grabber_obj_dist, limits=(0, self.GRABBER_OBJ_DIST_CLIP))
grabber_object_dist_reward = (self.GRABBER_OBJ_DIST_CLIP - clipped_grabber_obj_dist) / self.GRABBER_OBJ_DIST_CLIP  # Normalize to b/w 0 and 1

# Exponentiate to make high rewards more potent and weak rewards less potent
grabber_object_dist_reward = (grabber_object_dist_reward**self.GRABBER_OBJ_DIST_REWARD_EXP_FACTOR)

reward = obj_target_dist_reward + grabber_object_dist_reward
```

where

```
python
GRABBER_OBJ_DIST_REWARD_EXP_FACTOR = 2.5
```

Here the reward is computed as the sum of the distance of the claw (more specifically, the claw and its fingers to reward the closing movement of the fingers around the block) and the amount the block has moved to a target location (in other words, if the block has been picked up and moved upwards).

## Environment Setup
### Setting Up the Simulation
1. **Gazebo:**
   * URDF to define your robot model.
   * Create a world file with necessary obstacles and sensors.

#### Important: When launching in a gazebo simulation it starts in a paused state, you must unpase it and then sleep() for a short period of time.
```
python
def unpause_gazebo():
    rospy.wait_for_service("/gazebo/unpause_physics")
    unpause_gazebo = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
    unpause_gazebo()
```

#### Important: To have your simulation run faster than real time you must add the following to your .world file:
```
xml
<physics name="default_physics" type="ode">
    <real_time_factor>0</real_time_factor>  <!-- Real-time factor for simulation speed (1.0 = real time, 0 = max) -->
    <real_time_update_rate>0</real_time_update_rate>  <!-- Number of updates per second --> (0 = max)
</physics>

```

2. **Customizing the Gym Environment:**
   * Inherit from `gym.Env`.
   * Implement `reset()` to initialize the environment begining state.
   * Implement `step(action)` to execute the action, calculate rewards, and return new observations.
   * Use the ROS-Gazebo bridge (publishers and subscribers) to control the robot in Gazebo and receive sensor data.

---

## Visualization
* Use Gazebo to see your robot interacting with its environment, not recommended for actual prolonged training.
* Use RViz to monitor your robots behavior during training, as it is **much faster**. 

Launching in an RViz state typaically required to add the gui:=false use_rviz:=true flags to your launch file.

---

## Deployment and Testing
Once training is complete:
1. Load the saved model
2. Observe environment with sensors (subscribers)
3. Run `model.predict()` to get the action.
4. Publish the action.
5. Repeat 2-4 until task is accomplished.

...

For more specific implementation details, see the PX100_Pick_and_Place_SAC project repository.