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

To operate Baxter, a series of nodes is required. An interaction map of how the nodes in our package interact is shown below:

![Interaction Map](https://github.com/enginerd887/Baxter-Checkers-ME495_2016/blob/master/checkers/screenshots/FinalProjectFlow.png)

### How to Run the Demo
> roslaunch user_interface.launch

Then input your desired moves when prompted by the terminal.
Make sure both the square you want to move from and the square you would like to move to are included in the string entered.
Use this format: "B2 A3"

### Workflow Operation

  When Baxter plays against a human adversary, the code loops through the following steps:

  1. Human makes a move.
  All moves by the human opponent must be manually entered into the checkers_stretch node to allow Baxter to update the current state of the board (ex. "B2 A3").
  2. Baxter runs through the logic of the in-built checkers engine.
  Once the robot has decided what its next move will be, a string containing the coordinates of a piece's loation and its desired destination is published topic 'relay'. (ex. "E5 F4").
  3. The search node - which subscribes to 'relay' - translates the string into one of 18 hard-coded joint poses.
  Each pose is represented by an end effector point and a quaternion. After parsing the string, the cooresponding Pose() message with the appropriate (point, quaternion) combination is published to the '/desired_position/pose' topic.
  4. The ik_service_client subscribes to the Pose message published by the search node, and tries to find a solution to the IK problem. If a solution is found, the arm will subsequently move to that position via "set_joint_positions".
  5. Once the end effector arrives at the calculated location, the center_detection node will read all red objects captured by the camer in the left limb and attempt to calculate the centroid of the red region closest to the center of the camera view. Next, the node will pass this centroid data to the calc_and_send_pose node, which calculates the proportional control needed to adjust the (x,y) positions of the end effector. The obtained coordinates are then published as a Pose() to the 'center_of_piece' topic.
  6. Because the IK solver node also subscribes to this topic, Baxter will once again attempt to solve for the necessary joint angles and position Baxter's gripper directly over the block, with a finger on either side.
  7. Subsequently, the calc_and_send_pose function will close the gripper, receive the goal point of the block, and trigger the IK solver again to move to the goal square.
  8. Once the desired destination is reached, function calc_and_send_pose will open the gripper to release the block, completing the move. calc_pose will also call a "safety_config" and a "home_config", which will alert Baxter to raise the arm directly above the block, before returning to the home configuration. This safety feature was implemented to prevent the arm from colliding with other pieces on the board.
  9. Now that Baxter has completed his move he will wait for a new move from the human user, and the entire process is repeated again.

### Hardware Components
The [Baxter] robot by rethinkrobotics
-> Insert picture of baxter?
-> insert picture of board with pieces

### Software Components
#### Dependencies
    [baxter_interface]
    [cv_bridge]
    [rospy]
    [sensor_msgs]
    [std_msgs]

#### Topics


#### Nodes
    checkers_stretch.py
    search.py
    ik_service_client.py
    center_detection.py
    calc_and_send_pose.py

#### Launch Files
    checkers.launch
    user_interface.launch

### Troubleshooting
-> issues with color recognition. very sensitive to changes in lighting
-> grab issues...could've used the stuff jarvis talked about...uh in-built distance sensor. effect of fix may depend on size of object

### Resulting Performance

### Future Direction

### Concluding Remarks

[Baxter]: http://www.rethinkrobotics.com/baxter/
[baxter_interface]: http://sdk.rethinkrobotics.com/wiki/Baxter_Interface
[cv_bridge]: http://wiki.ros.org/cv_bridge
[rospy]: http://wiki.ros.org/rospy
[sensor_msgs]: http://wiki.ros.org/sensor_msgs
[std_msgs]: http://wiki.ros.org/std_msgs
