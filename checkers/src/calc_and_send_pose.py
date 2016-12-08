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

# Global variables
pose_ee = np.full((7,1), None)
position_Piece = Point()

# All my local flags
got_current_pose = False 
got_center_position = False 


# Create publisher to send PoseStamped() message to topic for Baxter to move towards
pub_baxtermovement = rospy.Publisher('desired_position/pose', Pose, queue_size=10)


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
    #rospy.loginfo("Got center position...")

 
def poseToGrip(msg):
    '''CALLBACK: Creates PoseStamped() message to move towards piece and grip it '''
    
    if msg: 

        rospy.loginfo("Calculating new pose...")

        # Only calc new pose if we're ready
        if got_current_pose and got_center_position:         

            # Create a Pose() message based on piece position and current ee pose
            move_to_pose = Pose()
            #move_to_pose.header=Header(stamp=rospy.Time.now(), frame_id='base')
            move_to_pose.pose.position=Point(
                x = position_Piece.x,
                y = position_Piece.y,
                z = pose_ee[2,0] - distance_above,
                    )
            move_to_pose.pose.orientation=Quaternion(
                x = pose_ee[3,0],
                y = pose_ee[4,0],
                z = pose_ee[5,0],
                w = pose_ee[6,0],
                    )
        
            # Send PoseStamped() message to Baxter's movement function
            pub_baxtermovement.publish(move_to_pose)

            # Change flag to let them know I'm done
            pub_flag.publish(False)
            rospy.loginfo("Pose sent...")         


# Main sequence of events
def main():

    rospy.init_node('calc_and_send_pose')

    # Subscribe to the topic which lets me know when to start
    rospy.Subscriber("got_to_position", Bool, poseToGrip)

    # Also make a publisher to let them know I'm done
    pub_flag = rospy.Publisher("got_to_position", Bool, queue_size=10)

    # Subscribe to Baxter's left ee pose and center of piece topic
    rospy.Subscriber("/robot/limb/left/endpoint_state",EndpointState,readEEPose)
    rospy.Subscriber("center_of_piece",Point,getPiecePosition)

    # dummy pose
    dummy_pose = Pose(
                position=Point(x= 0.460597556131,y=0.215572057988,z=-0.124597871731),
                orientation=Quaternion(x= 0.998658511565,y=-0.00795861100836,z=-0.00425494800061,w=-0.0509875789477)
            )
    
    pub_baxtermovement.publish(dummy_pose)
    rospy.loginfo("Dummy Pose sent...")

    rospy.spin()



if __name__ == '__main__':
    main()
