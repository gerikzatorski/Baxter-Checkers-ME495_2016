#!/usr/bin/env python

import rospy
import baxter_interface
import numpy as np

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import String,Header,Bool

from baxter_core_msgs.msg import EndpointState

from math import fabs

# Global variables
pose_ee = np.full((7,1), None)
position_Piece = Point()

# All my local flags
got_current_pose = False 
got_center_position = False 


# Create publisher to send PoseStamped() message to topic for Baxter to move towards
pub_baxtermovement = rospy.Publisher('desired_position/pose', Pose, queue_size=10)
# Also make a publisher to let them know I'm done
pub_flag = rospy.Publisher("got_to_position", Bool, queue_size=10)
# Another pub
#pub_gripper = rospy.Publisher("gripper_state", Bool, queue_size=10)


def readEEPose(msg):
    ''' Callback fxn for reading the Pose() of the EE'''

    pose = msg.pose

    position_new = pose.position
    orientation_new = pose.orientation

    global pose_ee
    global got_current_pose

    pose_ee[0,0] = position_new.x
    pose_ee[1,0] = position_new.y
    pose_ee[2,0] = position_new.z

    pose_ee[3,0] = orientation_new.x
    pose_ee[4,0] = orientation_new.y
    pose_ee[5,0] = orientation_new.z
    pose_ee[6,0] = orientation_new.w

    got_current_pose = True
 

def getPiecePosition(msg):
    ''' Callback fxn for reading the position of the piece'''

    global position_Piece 
    position_Piece = msg

    global got_center_position
    got_center_position = True

 
def poseToGrip(msg):
    '''CALLBACK: Creates PoseStamped() message to move towards piece and grip it '''
    
    if msg: 

        # Only calc new pose if we're ready
        if got_current_pose and got_center_position:
          

            # Create a Pose() message based on piece position and current ee pose
            move_to_pose = Pose()

            incremental_distance =  0.0025
                 
            diffx = position_Piece.x - 23
            diffy = position_Piece.y - 83
	    scale_factor_x = fabs(diffx)*0.3
	    scale_factor_y = fabs(diffy)*0.4

            # X-position of point not within range of center of frame
            if diffx < 0:
                pointx = pose_ee[0,0] + incremental_distance*scale_factor_x
                
            elif diffx > 0:
                pointx = pose_ee[0,0] - incremental_distance*scale_factor_x
                
            else:
                pointx = pose_ee[0,0];

            # Y-position of point not within range of center of frame
            if diffy < 0:
                pointy = pose_ee[1,0] - incremental_distance*scale_factor_y
                
            elif diffy > 0:
                pointy = pose_ee[1,0] + incremental_distance*scale_factor_y
                
            else:             
                pointy = pose_ee[1,0]

            
            move_to_pose.position=Point(
                x = pointx, 
                y = pointy, 
                z = -0.19,
                    )
            move_to_pose.orientation=Quaternion(
                x = pose_ee[3,0],
                y = pose_ee[4,0],
                z = pose_ee[5,0],
                w = pose_ee[6,0],
                    )
        
            # Go DOWN
            pub_baxtermovement.publish(move_to_pose) 
            rospy.sleep(5)

            # GRAB Calibrate and close left gripper
            baxterleft = baxter_interface.Gripper('left')
            baxterleft.close()
                   


# Main sequence of events
def main():

    rospy.init_node('calc_and_send_pose')

    # Subscribe to the topic which lets me know when to start
    rospy.Subscriber("got_to_position", Bool, poseToGrip)

    # Subscribe to Baxter's left ee pose and center of piece topic
    rospy.Subscriber("/robot/limb/left/endpoint_state",EndpointState,readEEPose)
    rospy.Subscriber("center_of_piece",Point,getPiecePosition)


    rospy.spin()



if __name__ == '__main__':
    main()
