#!/usr/bin/env python

"""
Baxter RSDK Inverse Kinematics Example
"""
import argparse
import struct
import sys
import baxter_interface
import rospy
import numpy as np

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

# initialize desired pose
global desired_pose 
desired_pose = Pose()


def ik_test(limb, pose):
    rospy.init_node("rsdk_ik_service_client")
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        'left': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(x = pose.position.x,
                               y = pose.position.y,
                               z = pose.position.z),
                orientation=Quaternion(x = pose.orientation.x,
                                       y = pose.orientation.y,
                                       z = pose.orientation.z,
                                       w = pose.orientation.w),
            ),
        ),
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=0.656982770038,
                    y=-0.852598021641,
                    z=0.0388609422173,
                ),
                orientation=Quaternion(
                    x=0.367048116303,
                    y=0.885911751787,
                    z=-0.108908281936,
                    w=0.261868353356,
                ),
            ),
        ),
    }

    ikreq.pose_stamp.append(poses[limb])
    rospy.loginfo("Made it here!")
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    # Check if result valid, and type of seed ultimately used to get solution
    # convert rospy's string representation of uint8[]'s to int's
    resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                               resp.result_type)
    if (resp_seeds[0] != resp.RESULT_INVALID):
        seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp_seeds[0], 'None')
        print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
              (seed_str,))
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        print "\nIK Joint Solution:\n", limb_joints
        print "------------------"
        print "Response Message:\n", resp

        limb = baxter_interface.Limb('left')
        limb.move_to_joint_positions(limb_joints)
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")

    return 0

def callback(msg):
   global desired_pose
   desired_pose = msg


def main():
    """RSDK Inverse Kinematics Example """

    #arg_fmt = argparse.RawDescriptionHelpFormatter
    #parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     #description=main.__doc__)
    #parser.add_argument(
    #    '-l', '--limb', choices=['left', 'right'], required=True,
    #    help="the limb to test"
    #)
    #args = parser.parse_args(rospy.myargv()[1:])
    
    limb = 'left'

    # Create subscriber
    rospy.Subscriber('desired_position/pose', Pose, callback)

    return ik_test(limb, desired_pose)

if __name__ == '__main__':
    sys.exit(main())
