# Baxter Checkers, ME 495 Fall 2016
## Baxter the Robot learns the classic game of skill.
-------------------------------------------
![Team Shot](https://github.com/enginerd887/Baxter-Checkers-ME495_2016/blob/master/checkers/screenshots/CheckersTeam.png)

## Team 2
-------------------------------------------
>Gale Curry

>Stephanie Chang

>Alfonso (Tito) Fernandez

>Gerik Zatorski

### Introduction

The goal of this project was to get a Baxter device to play checkers (a.k.a. droughts) against a human opponent. To achieve this, a combination of inverse kinematics, camera detection, and proportional control was implemented.

![Baxter Ready](https://github.com/enginerd887/Baxter-Checkers-ME495_2016/blob/master/checkers/screenshots/BaxterCheckersReady.JPG)

The code contained in this repository successfully grants Baxter the ability to autonomously compete against a human rival. A brief summary of how the <b>checkers</b> package works is provided below.

### Materials
The game is played on a home-made 6x6 checkerboard, with custom pieces. The human pieces are pieces of 1x3 wood painted white, and Baxter's pieces are wooden cubes covered in red painter's tape.

![Board](https://github.com/enginerd887/Baxter-Checkers-ME495_2016/blob/master/checkers/screenshots/CheckerBoard.png)

The board itself is composed of 3"x3" squres. The white pieces are 2.5" square, 3/4" deep, and the red pieces are all 2" cubes. Note that cubes were chosen for Baxter's pieces because they give the gripper a larger surface area to work with.

### Node Network

To operate Baxter, a series of nodes is required. An interaction map of how the nodes in our package interact is shown below:

![Interaction Map](https://github.com/enginerd887/Baxter-Checkers-ME495_2016/blob/master/checkers/screenshots/FinalProjectFlow.png)

### How to Run the Demo
> roslaunch user_interface.launch

Input your desired moves when prompted by the terminal.<br>
Make sure both the square you want to move from and the square you would like to move to are included in the string entered.<br>
Ex. If you want to move from B2 to A3, stick to this format: B2 A3

Watch a sneak preview of Baxter making a move against a human opponent!
https://vimeo.com/195051138

### Workflow Operation

![Baxter against Allen](https://github.com/enginerd887/Baxter-Checkers-ME495_2016/blob/master/checkers/screenshots/BaxterandAllen.png)
  When Baxter plays against a human adversary, the code loops through the following steps:

  1. Human makes a move.
  All moves by the human opponent must be manually entered into the checkers_stretch node to allow Baxter to update the current state of the board (ex. "B2 A3").
  2. Baxter runs through the logic of the in-built checkers engine.
  Once the robot has decided what its next move will be, a string containing the coordinates of a piece's location and its desired destination is published to topic 'relay'. (ex. "E5 F4").
  3. The search node - which subscribes to 'relay' - translates the string into one of 18 hard-coded joint poses.
  Each pose is represented by an end effector point and a quaternion. After parsing the string, the corresponding Pose() message with the appropriate (point, quaternion) combination is published to the '/desired_position/pose' topic.
  4. The ik_service_client subscribes to the Pose message published by the search node, and tries to find a solution to the IK problem. If a solution is found, the arm will subsequently move to that position via "set_joint_positions".
  5. Once the end effector arrives at the calculated location, the center_detection node will read all red objects captured by the camera in the left limb and attempt to calculate the centroid of the red region closest to the center of the camera view. Next, the node will pass this centroid data to the calc_and_send_pose node, which calculates the proportional control needed to adjust the (x,y) positions of the end effector. The obtained coordinates are then published as a Pose() to the 'center_of_piece' topic.
  6. Because the IK solver node also subscribes to this topic, Baxter will once again attempt to solve for the necessary joint angles and position Baxter's gripper directly over the block, with a finger on either side.
  7. Subsequently, the calc_and_send_pose function will close the gripper, receive the goal point of the block, and trigger the IK solver again to move to the goal square.
  8. Once the desired destination is reached, function calc_and_send_pose will open the gripper to release the block, completing the move. calc_pose will also call a "safety_config" and a "home_config", which will alert Baxter to raise the arm directly above the block, before returning to the home configuration. This safety feature was implemented to prevent the arm from colliding with other pieces on the board.
  9. When Baxter completes a move, he will wait for his opponent to enter his or her response. And the entire process is repeated again.

### Hardware Components
  1. The [Baxter] robot by rethinkrobotics<br>
  2. A checker board and sufficiently large playing pieces with known dimensions. Note, A customized board and set of pieces was created for this project.<br>
-> insert picture of board with pieces

### Software Components
#### Dependencies
  [baxter_interface]<br>
  [cv_bridge]<br>
  [rospy]<br>
  [sensor_msgs]<br>
  [std_msgs]<br>

#### Topics
  Listed using the following format: 'Name', Type

    '/keys', String
    '/relay', String
    '/desired_position/pose', Pose
    '/got_to_position', Bool
    '/cameras/left_hand_camera/image', Image
    '/center_of_piece', Point
    '/robot/limb/left/endpoint_state', EndpointState

#### Nodes
  [checkers_stretch.py]<br>
  [search.py]<br>
  [ik_service_client.py]<br>
  [center_detection.py]<br>
  [calc_and_send_pose.py]<br>

#### Launch Files

To run the program, the user needs merely to type:
    [roslaunch checkers user_interface.launch]
This will run all nodes, getting Baxter ready to play. The user will see the camera image, an image showing where Baxter sees red, an output terminal from the IK solver indicating whether or not valid solutions were found, and a main terminal where the user inputs the human moves.

  [user_interface.launch]<br>

Note, user_interface.launch is the master launch file which starts up all of the nodes.

### Troubleshooting
Baxter's ability to select and accordingly pick up a particular block highly depends on the ambient lighting in the arena.

issues with color recognition. very sensitive to changes in lighting
-> grab issues...could've used the stuff jarvis talked about...uh in-built distance sensor. effect of fix may depend on size of object
>>>>>>> checkers/master

### Resulting Performance
Overall,
### Future Direction

### Concluding Remarks

[Baxter]: http://www.rethinkrobotics.com/baxter/
[baxter_interface]: http://sdk.rethinkrobotics.com/wiki/Baxter_Interface
[cv_bridge]: http://wiki.ros.org/cv_bridge
[rospy]: http://wiki.ros.org/rospy
[sensor_msgs]: http://wiki.ros.org/sensor_msgs
[std_msgs]: http://wiki.ros.org/std_msgs
[checkers_stretch.py]: https://github.com/enginerd887/Baxter-Checkers-ME495_2016/blob/master/checkers/src/checkers.py
[search.py]: https://github.com/enginerd887/Baxter-Checkers-ME495_2016/blob/master/checkers/src/search.py
[ik_service_client.py]: https://github.com/enginerd887/Baxter-Checkers-ME495_2016/blob/master/checkers/src/ik_service_client.py
[center_detection.py]: https://github.com/enginerd887/Baxter-Checkers-ME495_2016/blob/master/checkers/src/center_detection.py
[calc_and_send_pose.py]: https://github.com/enginerd887/Baxter-Checkers-ME495_2016/blob/master/checkers/src/calc_and_send_pose.py
[checkers.launch]: https://github.com/enginerd887/Baxter-Checkers-ME495_2016/blob/master/checkers/launch/checkers.launch
[user_interface.launch]: https://github.com/enginerd887/Baxter-Checkers-ME495_2016/blob/master/checkers/launch/user_interface.launch
