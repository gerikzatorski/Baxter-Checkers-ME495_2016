#!/usr/bin/env python

import rospy
import baxter_interface
from std_msgs.msg import Header, String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import String,Header,Bool

coord = [] 

def map_fxn(des_pos):
    global coord 
    #Hardcoded Checker Square Positions [position xyz, quaternion xyzw]
    des_pos = str(des_pos)

    col = [des_pos[6], des_pos[10]]

    row = [des_pos[8], des_pos[12]]
     
    for i in range(2):
        if col[i] is "A":
            if row[i] is "1":
                goal_config = [0.872938148433,-0.138781518623,-0.06, 0.988878629836,-0.029651083868,0.145533214769,0.00774287273203]
            elif row[i] is "3":
                goal_config = [0.729672658698,-0.14944781132,-0.06,0.999609426012,0.00572608810826,-0.00460464166538,-0.0269630231691]
            elif row[i] is "5":
                goal_config = [0.570712645075,-0.160234803396,-0.132486596368,1.0,0.0,0.0,0.0]

        elif col[i] is "B":
            if row[i] is "2":
                goal_config = [0.80180537159,-0.0709625138,-0.03,0.997055866064,-0.00804099205134,0.0495704603205,-0.0579457665175]
            elif row[i] is "4":
                goal_config = [0.659729579857,-0.0695190326657,-0.105100809599,1.0,0.0,0.0,0.0]
            elif row[i] is "6":
                goal_config = [0.48504435789,-0.0836423684025,-0.07,1.0,0.0, 0.0, 0.0]

        elif col[i] is "C":
            if row[i] is "1":
                goal_config = [0.871394050868,0.00321779590923,-0.06,-0.115830906904,0.0266418026216,0.0784627927274,-0.0197168440116]
            elif row[i] is "3":
                goal_config = [0.72099597135,0.00972967930975,-0.07, 0.997963425222, 0.0110519031264,0.0102812076764,-0.0619770451548]
            elif row[i] is "5":
                goal_config = [0.567421129759,-0.00370920108952,-0.05, 1.0,0.0,0.0,0.0]

        elif col[i] is "D":
            if row[i] is "2":
                goal_config = [0.801959617162,0.0919395299359,-0.05,0.998050141472,0.0411114433227,-0.0160781721526,-0.0441277318196]
            elif row[i] is "4":
                goal_config = [ 0.64274825762,0.0911833388399, -0.07,1.0,0.0,0.0,0.0]
            elif row[i] is "6":
                goal_config = [0.487570022135,0.0726773146802,-0.08, 1.0,0.0,0.0,0.0]

        elif col[i] is "E":
            if row[i] is "1":
                goal_config = [0.881350899802,0.170378242711,-0.06,0.996251867923,0.0471189426062,0.0419025772193,-0.0592131313968]
            elif row[i] is "3":
                goal_config = [0.711382933155,0.169900893084,-0.06,0.996752095131,0.0399645409214,-0.0107386476991,-0.0690852934121]
            elif row[i] is "5":
                goal_config = [0.558785280222,0.147260499487,-0.08, 1.0,0.0,0.0,0.0]

        elif col[i] is "F":
            if row[i] is "2":
                goal_config = [0.805619812179, 0.246316555643,-0.05,0.997019289535,0.0484012010984,-0.0236201512055,-0.0552444430097]
            elif row[i] is "4":
                goal_config = [0.634406807152,0.234400486975,-0.07,1.0, 0.0,0.0,0.0]
            elif row[i] is "6":
                goal_config = [0.460597556131,0.215572057988,-0.07, 1.0,0.0,0.0,0.0]
        
        coord.append(goal_config)

    fromconfig=Pose(
        position=Point(
            x=coord[0][0],
            y=coord[0][1],
            z=coord[0][2],
        ),
        orientation=Quaternion(
            x=coord[0][3],
            y=coord[0][4],
            z=coord[0][5],
            w=coord[0][6],
        ))

    toconfig=Pose(
        position=Point(
            x=coord[1][0],
            y=coord[1][1],
            z=coord[1][2],
        ),
        orientation=Quaternion(
            x=coord[1][3],
            y=coord[1][4],
            z=coord[1][5],
            w=coord[1][6],
        ))

    # FIRST Calibrate and open left gripper
    baxterleft = baxter_interface.Gripper('left')
    baxterleft.calibrate()
    baxterleft.open()
    pub.publish(fromconfig)

    # wait for 10 sec (while camera runs)
    rospy.sleep(10)

    # send got_to_position = True flag to Gale's node
    pub2.publish(True)

    # wait again
    rospy.sleep(7)
    pub.publish(toconfig)
    
    # wait 
    rospy.sleep(7)

    drop_config=Pose(
        position=Point(
            x=coord[1][0],
            y=coord[1][1],
            z=-0.19,
        ),
        orientation=Quaternion(
            x=coord[1][3],
            y=coord[1][4],
            z=coord[1][5],
            w=coord[1][6],
        ))    
    pub.publish(drop_config)
    rospy.sleep(5)

    # RELEASE: open left gripper
    baxterleft = baxter_interface.Gripper('left')
    baxterleft.open()
    rospy.sleep(5)

    # safety plane
    safety_config=Pose(
        position=Point(
            x=coord[1][0],
            y=coord[1][1],
            z=-0.1,
        ),
        orientation=Quaternion(
            x=coord[1][3],
            y=coord[1][4],
            z=coord[1][5],
            w=coord[1][6],
        ))  
    pub.publish(safety_config)
    rospy.sleep(5)

    # GO HOME
    home_config=Pose(
        position=Point(
            x= 0.464103257539,
            y= 0.12055341652,
            z= -0.0435690126601,
        ),
        orientation=Quaternion(
            x= 1.0,
            y= 0.0,
            z= 0.0,
            w= 0.0,
        ))    
    pub.publish(home_config)
    
    coord = []
        

if __name__ == '__main__':

    rospy.init_node('search')
    sub = rospy.Subscriber('relay', String, map_fxn)
    pub = rospy.Publisher('/desired_position/pose', Pose, queue_size = 10)
    pub2 = rospy.Publisher("got_to_position", Bool, queue_size = 10)   


    rospy.spin()

