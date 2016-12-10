# ME 495 Final Project (Fall 2016)
## RoboCheckers: Baxter learns the classic game of skill.
-------------------------------------------
![Team Shot](https://github.com/enginerd887/Baxter-Checkers-ME495_2016/blob/master/checkers/screenshots/CheckersTeam.png)

## Team 2
-------------------------------------------
>Gale Curry | Stephanie Chang | Alfonso (Tito) Fernandez | Gerik Zatorski

### Introduction

The goal of this project was to get a Baxter device to play checkers (a.k.a. droughts) against a human opponent. To achieve this, a combination of inverse kinematics, camera detection, and proportional control was implemented.

![Baxter Ready](https://github.com/enginerd887/Baxter-Checkers-ME495_2016/blob/master/checkers/screenshots/BaxterCheckersReady.JPG)

The code contained in this repository successfully grants Baxter the ability to autonomously compete against a human rival.

### How to Run the Demo
> roslaunch user_interface.launch

Input your desired moves when prompted by the terminal.<br>
Make sure both the square you want to move from and the square you would like to move to are included in the string entered.<br>
Ex. If you want to move from B2 to A3, stick to this format: B2 A3

Click on the picture to watch a sneak preview of Baxter making a move against a human opponent!

[![Screenshot](https://github.com/enginerd887/Baxter-Checkers-ME495_2016/blob/master/checkers/screenshots/BaxterScreenshot.png)](https://vimeo.com/195051138)

### Node Network

To operate Baxter, a series of nodes is required. An interaction map of how the nodes in our package interact is shown below:

![Interaction Map](https://github.com/enginerd887/Baxter-Checkers-ME495_2016/blob/master/checkers/screenshots/FinalProjectFlow.png)

### Workflow Operation

![Baxter against Alan](https://github.com/enginerd887/Baxter-Checkers-ME495_2016/blob/master/checkers/screenshots/BaxterandAllen.png)

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

The game is played on a home-made 6x6 checkerboard, with custom pieces. The human pieces are pieces of 1x3 wood painted white, and Baxter's pieces are wooden cubes covered in red painter's tape. The pieces were originally painted red, but it was more difficult for Baxter's camera to see that particular shade, so we switched to painter's tape.

![Board](https://github.com/enginerd887/Baxter-Checkers-ME495_2016/blob/master/checkers/screenshots/CheckerBoard.png)

The board itself is composed of 3"x3" squares, with a 1" silver border. The white pieces are 2.5" square, 3/4" deep, and the red pieces are all 2" cubes. Note that cubes were chosen for Baxter's pieces because they give the gripper a larger surface area to work with.

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

  [user_interface.launch]<br>

As mentioned above, to run the program, the user needs merely to type:
> roslaunch checkers user_interface.launch

This will run all nodes, getting Baxter ready to play. The user will see the camera image, a binary mask showing where Baxter sees red, an output terminal from the IK solver indicating whether or not valid solutions were found, and a main terminal where the user inputs the human moves.

NOTE: the IK node used in this project (ik_service_client.py) is the IK service from the baxter examples provided by Rethink Robotics. The only modification that was made was that, once the solver solves the IK and finds a solution, *set_joint_positions* was called to get Baxter to move to the position found by the solver. In future iterations, we intend to write a more robust IK node that iteratively calls Baxter's IK service, adds noise to the start and target positions, and calculates a series of poses between start and goal positions in order to make it more likely for a solution to be found. Alternatively, we could figure out how to incorporate MoveIt into our project for more reliable moves.

###Performance
Baxter's newly discovered proficiency at playing checkers is the product of a harmonious marriage between the checkers logic and image recognition scripts. While [checkers_stretch.py] shoulders the bulk of the "intelligence" needed for Baxter to follow the rules of checkers, recognize kinging, and execute double jump moves, [center_detection.py] is what helps the robot target specific pieces to move.

Click the picture to see Baxter square off against MSR student Alan Hong:

[![Screenshot2](https://github.com/enginerd887/Baxter-Checkers-ME495_2016/blob/master/checkers/screenshots/FullMovieTitle.png)](https://vimeo.com/195063628)

#### Checkers Logic
Originally, the checkers logic was performed by [checkers.py], a home-made checkers engine which did not include more advanced concepts such as kinging and double capture. This was eventually updated to produce [checkers_stretch.py], which implements a full checkers engine, developed by our team.

In the [checkers_stretch.py] code, several enum-like structures are defined to help the robot know what is happening in the game. For example, the *GameState* class defines the current point in the game (is it red's turn, white's turn, etc.),and the *TileState* determines whether a given square is empty, contains a red or white piece, contains a red or white king, or is not a valid place to be on the board. It then defines a *CheckersGame* class, a *CheckersBoard* class, and a *Tile* class to define the checkers environment in the robot's mind, and implements a series of error checks to make sure that both the human and the robot only choose valid moves.

To choose a move, Baxter checks the current state of the board for any possible captures. If one is available, it takes the capture. If a capture is taken, it checks to see if a sequential capture can occur, and takes it if present. Otherwise, Baxter's turn consists basically of scanning the board for valid moves, and then randomly selecting a valid move to make.

The end result is that Baxter is guaranteed to take a capture if one is present. Otherwise, it plays the game like a young child, following the rules without considering strategy. Future work includes adding a bit of artificial intelligence to this node, possibly by  scanning for legal moves, assigning each move a weight based on factors like immediate counter-capture or taking a double vs. a single jump.

#### Image Recognition
In the center_detection.py node we used the OpenCV library to detect the pieces for Baxter to pick up. First, Baxter moves his arm over the general area of the piece so it is in view of the camera, then this image is converted into HSV format. The image is then thresholded to only include pixels within our defined HSV range for red, resulting in a binary mask. Since red straddles the beginning and the end of the spectrum, two separate ranges are applied and the two results are added together. This image is then filtered again to remove noise, and finally the contours of the resulting image are detected. To choose the contour corresponding to a checkers piece, the contours with small areas are ignored. Since many pieces may be in the view of the camera at once, Baxter now calculates the center of mass of each contour and chooses the one which is closest to the center of the image. This is under the assumption that Baxter already has moved his gripper close enough to the desired piece. This chosen center of mass (x,y) is then sent off to the calc_and_send_pose.py node to help Baxter plan his motion to grip the piece. In future iterations we would like to include the orientation of the piece to make his gripping more robust.

### Troubleshooting & Future Direction
Baxter's ability to select and accordingly pick up a particular block highly depends on the ambient lighting in the arena. This is because our package relies on color recognition (RED!) coupled with contour area estimation to single out potential objects to move. During testing, slight variations in how bright or dark different red objects also impacted the accuracy of the color filter. More vibrant shades were received favorably. To fix this issue, we attempted adjusting the HSV range of acceptable red values. However, some issues still occur. We realized afterward that red is perhaps the most difficult color to recognize due to the nature of HSV. We believe that replacing the red blocks with blocks of a different color, such as green, would have a positive effect on Baxter's performance.

Currently, our code only makes use of Baxter's left arm to manipulate objects. As a result, there are positions on the board where the ik_service_client has difficulty finding solutions. To improve Baxter's reach and mobility, the right arm could be enabled when goal positions are on the right hand side of the board. In addition, path planning a series of sequential positions from start to end pose would help the IK solver find solutions. Adding some noise to the initial position and iteratively calling the solver are also possible solutions to help Baxter reach the unreachable tiles on the board.

To refine the method with which Baxter grabs blocks, the distance sensors built into the arms could also be used. Also, at the moment Baxter plans his motion once from the hardcoded position above the board to the position atop the block. To make it more likely for the gripper to get the block, the program could iteratively plan and correct the path a few times as the gripper goes down.



### Concluding Remarks
This package was created for Professor Jarvis Schultz's ME 495 introductory course to ROS. It is the cumulative result of a quarter's worth of learning.

While the class may now be over, our group plans on integrating more improvements so Baxter can one day be as proficient as Chinook, the machine which beat Checkers Grandmaster Marion Tinsley. Maybe.

Last edited: December 9, 2016

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
