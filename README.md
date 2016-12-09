# Baxter Checkers, ME 495 Fall 2016
## Baxter the Robot learns the classic game of skill.
-------------------------------------------
## Team 2
-------------------------------------------
>Gale Curry

>Stephanie Chang

>Alfonso (Tito) Fernandez

>Gerik Zatorski

### Introduction

The goal of this project was to get a Baxter device to play checkers (a.k.a. droughts) against a human opponent. To achieve this, a combination of inverse kinematics, camera detection, and proportional control was implemented.

The code contained in this repository successfully grants Baxter the ability to autonomously compete against a human rival. A brief summary of how the <b>checkers</b> package works is provided below.

### Node Network

To operate Baxter, a series of nodes is required. The interaction map of how the nodes in our package interact is shown below:

![Interaction Map](https://github.com/enginerd887/Baxter-Checkers-ME495_2016/blob/master/checkers/screenshots/FinalProjectFlow.png)

### Workflow Operation

  With a human adversary, the nodes work together as follows:

  1. Human makes a move, the move is inputted into the checkers_stretch node to allow Baxter to update the current state of the board (ex. "B2 A3").
  2. Baxter runs through the logic of the in-built checkers engine to decide what his next move should be. He outputs the result as a string to the search node (ex. "E5 F4").
  3. The search node translates the string into one of 18 hard-coded joint poses, represented by an end effector point and a quaternion. The search node then publishes a Pose() message with the appropriate (point, quaternion) combination to the '/desired_position/pose' topic.
  4. The ik_service_client subscribes to the Pose message published by the search node, and tries to find a solution to the IK problem. If a solution is found, the arm moves to that position via "set_joint_positions".
  5. Once the end effector arrives at the calculated location, the center_detection node reads all the red in the left limb camera and attempts to calculate the centroid of the red region closest to the center of the camera view. It then passes this centroid data to the calc_and_send_pose node, which, as the name implies, calculates the proportional control needed to adjust the (x,y) positions of the end effector, then publishes a Pose() to the **blah** topic.
  6. Since the IK solver is subscribing to this topic, it now attempts to solve and move to the new position, which will position Baxter's gripper directly over the block, with a finger on either side.
  7. The calc_and_send_pose function now closes the gripper, receives the goal point of the block , and triggers the IK solver again to move to the goal square.
  8. Once at the new position, the calc_and_send_pose function opens the gripper to release the block, completing the move. The calc_pose function again calls a "safety_config" and a "home_config", raising the arm directly above the block to avoid collisions and then returning to the home configuration.
  9. Baxter has now completed his move and awaits a new move from the human user.

### Software Components

#### Dependencies

#### Topics

#### Nodes

### Resulting Performance

### Future Direction

### Concluding Remarks

