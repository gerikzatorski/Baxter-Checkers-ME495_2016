#!/usr/bin/env python

import rospy
from std_msgs.msg import Header, String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

coord = [] 

def map_fxn(des_pos):
    global coord 
    #Hardcoded Checker Square Positions [position xyz, quaternion xyzw]
    des_pos = str(des_pos)
    print(des_pos)
    col = [des_pos[6], des_pos[10]]
    print(col)
    row = [des_pos[8], des_pos[12]]
     
    for i in range(2):
        if col[i] is "A":
            if row[i] is "1":
                goal_config = [0.872938148433,-0.138781518623,-0.110217993585, 0.988878629836,-0.029651083868,0.145533214769,0.00774287273203]
            elif row[i] is "3":
                goal_config = [0.729672658698,-0.14944781132,-0.120499311968,0.999609426012,0.00572608810826,-0.00460464166538,-0.0269630231691]
            elif row[i] is "5":
                goal_config = [0.570712645075,-0.160234803396,-0.132486596368,0.998810315661,0.00349512257207,0.0118907556165,-0.0471629873764]

        elif col[i] is "B":
            if row[i] is "2":
                goal_config = [0.80180537159,-0.0709625138,-0.0894859550322,0.997055866064,-0.00804099205134,0.0495704603205,-0.0579457665175]
            elif row[i] is "4":
                goal_config = [0.659729579857,-0.0695190326657,-0.105100809599,0.998243992008, 0.00733016177506,-0.0118702144719,-0.0575699501205]
            elif row[i] is "6":
                goal_config = [0.48504435789,-0.0836423684025,-0.130506087796,0.999855544347,-0.00267882923027,-0.00779495718922, 0.0148644863958]

        elif col[i] is "C":
            if row[i] is "1":
                goal_config = [0.871394050868,0.00321779590923,-0.115830906904,-0.115830906904,0.0266418026216,0.0784627927274,-0.0197168440116]
            elif row[i] is "3":
                goal_config = [0.72099597135,0.00972967930975,-0.12214094892, 0.997963425222, 0.0110519031264,0.0102812076764,-0.0619770451548]
            elif row[i] is "5":
                goal_config = [0.567421129759,-0.00370920108952,-0.105067568307, 0.999429191406,0.00268921746424,0.00968404110811,-0.0322533536521]

        elif col[i] is "D":
            if row[i] is "2":
                goal_config = [0.801959617162,0.0919395299359,-0.108423071782,0.998050141472,0.0411114433227,-0.0160781721526,-0.0441277318196]
            elif row[i] is "4":
                goal_config = [ 0.64274825762,0.0911833388399, -0.125320774952,0.996796705079,0.0276299640955,-0.00692756602472,-0.0747323401018]
            elif row[i] is "6":
                goal_config = [0.487570022135,0.0726773146802,-0.129476251423, 0.998788771531,-0.0137780887136,-0.03460602248,-0.0321492977016]

        elif col[i] is "E":
            if row[i] is "1":
                goal_config = [0.881350899802,0.170378242711,-0.109550135653,0.996251867923,0.0471189426062,0.0419025772193,-0.0592131313968]
            elif row[i] is "3":
                goal_config = [0.711382933155,0.169900893084,-0.111987467592,0.996752095131,0.0399645409214,-0.0107386476991,-0.0690852934121]
            elif row[i] is "5":
                goal_config = [0.558785280222,0.147260499487,-0.137364332417, 0.997673930501,0.0192920144688,-0.0412714112351,-0.0507071710071]

        elif col[i] is "F":
            if row[i] is "2":
                goal_config = [0.805619812179, 0.246316555643,-0.101467056386,0.997019289535,0.0484012010984,-0.0236201512055,-0.0552444430097]
            elif row[i] is "4":
                goal_config = [0.634406807152,0.234400486975,-0.122325653523,0.996848785384, 0.0281205623789,-0.026556072908,-0.0692568266858]
            elif row[i] is "6":
                goal_config = [0.460597556131,0.215572057988,-0.124597871731, 0.998658511565,-0.00795861100836,-0.00425494800061,-0.0509875789477]
        
        coord.append(goal_config)
    print(coord)
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

    pub.publish(fromconfig)
    #print(fromconfig)
        

if __name__ == '__main__':

    rospy.loginfo("Searching for goal configuration...")
    rospy.init_node('search')
    sub = rospy.Subscriber('relay', String, map_fxn)
    pub = rospy.Publisher('/desired_position/pose', Pose, queue_size = 10)   

    #map_fxn("A 3 B 2")

    #while not rospy.is_shutdown():   

        #rospy.sleep(5)
        #pub.publish(toconfig)

    rospy.spin()

