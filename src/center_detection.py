#!/usr/bin/env python

import rospy
import baxter_interface
import numpy as np
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
from math import fabs

# HSV thresholding limits for RED
low_red = np.array([0,50,50])
high_red = np.array([10,255,255])
low_red_2 = np.array([150,50,50])
high_red_2 = np.array([179,255,255])

# A publisher to publish center (x, y) of red square
pub = rospy.Publisher('center_of_piece', Point, queue_size=10)

 
def callback(ros_img):
    ''' This function takes a ros image, converts it to an openCV image, then thresholds it, filters it and detects contours in 	  the shape of rects. It chooses the rect closest to the center of the image, and publishes the center of that rect as a Point(x,y). ''' 

    # Create "bridge" between ros and opencv and convert image
    bridge = CvBridge()
    try:
      cv_image = bridge.imgmsg_to_cv2(ros_img, "bgr8") # (color image with blue-green-red color order)

    except CvBridgeError as e:
      print(e)

    # Height/width used later for coordinate transform	
    height, width, depth = cv_image.shape 

    # Convert image to HSV format then to binary mask
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    binary_mask = cv2.inRange(hsv_image, low_red, high_red) # returns a binary mask, where white px: in range, black: not in range 
    binary_mask_2 = cv2.inRange(hsv_image, low_red_2, high_red_2)
    binary_mask_total = cv2.bitwise_or(binary_mask, binary_mask)

    # Filtering the image
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
    binary_mask_total = cv2.morphologyEx(binary_mask_total, cv2.MORPH_OPEN, kernel)
    binary_mask_total = cv2.morphologyEx(binary_mask_total, cv2.MORPH_CLOSE, kernel)

    # Threshold and find edges (contours) of red piece
    retVal,thresholded_img = cv2.threshold(binary_mask_total,157,255,0)
    contours, hierarchy = cv2.findContours(thresholded_img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    # contours is a Python list of all the contours in the image. Each individual contour is a Numpy array of (x,y) coordinates   
    # of boundary points of the object.

    # Filters out small contours
    pieces_list = []
    for i in range(len(contours)):
        cont = contours[i] 
        area = cv2.contourArea(cont)
        
        if area > 3000:
            #rospy.loginfo("Area: % i", area)
            pieces_list.append(cont)

    # Choose the one closest to the center
    highest = 0
    best = None
    best_cont = None

    for cont in pieces_list: 
            rect = cv2.minAreaRect(cont)
            box = cv2.cv.BoxPoints(rect)
            box = np.int0(box)
            # Get center of square
            center_x, center_y = square_center_calc(box)
            if center_y > highest:
                dist_from_center = fabs(center_x - (width/2))
                #print(dist_from_center)
                if dist_from_center < 100:
                    best = box
                    best_cont = cont
            
    if best is not None and best_cont is not None:  
        cv2.drawContours(cv_image,[best],0,(0,0,255),2)            
    
        # Get center of square
        #center_x, center_y = square_center_calc(best)
        #cv2.circle(cv_image, (center_x, center_y), 5, (0, 255, 255), -1 )

        M = cv2.moments(best_cont)
        area = M['m00']
        if area>500:
		center_x = int(M['m10']/M['m00'])
		center_y = int(M['m01']/M['m00'])
    
        # Get orientation of square
        #(x,y),(MA,ma),angle = cv2.fitEllipse(cont)

        # Publish point 
        P = Point()
        P.x = center_x-(width)/2
        P.y = -(center_y - (height/2))
        pub.publish(P)

    # Show images on screen
    cv2.imshow("Original Image", cv_image)
    cv2.imshow("Thresholded Image", thresholded_img)
    cv2.imshow("Binary Mask", binary_mask_total)

    cv2.waitKey(3) 

 
def listener():
    '''Listens to the left hand camera'''

    #rospy.init_node('center_detection')

    # Creates windows for images
    cv2.namedWindow("Original Image", 1)
    cv2.namedWindow("Thresholded Image", 2)

    # Init Baxter's left hand camera node
    rospy.init_node('left_hand_camera', anonymous=True)

    # Create subscriber to grab image from camera feed 
    rospy.Subscriber("/cameras/left_hand_camera/image", Image, callback)

    # Don't exit
    rospy.spin()

def square_center_calc(box):
    ''' Takes the four corner coordinates of a BoxPoints and returns the coordinates of the center of the square''' 

    cx = (box[0][0] + box[2][0])/2;
    cy = (box[0][1] + box[2][1])/2; 

    return(cx, cy)

if __name__ == '__main__':
     listener()

