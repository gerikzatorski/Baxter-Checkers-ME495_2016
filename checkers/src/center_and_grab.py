#!/usr/bin/env python

import rospy
import baxter_interface
import numpy as np
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point

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
    
    cont = contours[0] # hope that the first contour is the right one?  
    rect = cv2.minAreaRect(cont) 
    box = cv2.cv.BoxPoints(rect)
    box = np.int0(box) 
    cv2.drawContours(cv_image,[box],0,(0,0,255),2)
    
    cont2 = contours[1] # try second one
    rect2 = cv2.minAreaRect(cont2)
    box2 = cv2.cv.BoxPoints(rect2)
    box2 = np.int0(box2)
    cv2.drawContours(cv_image,[box2],0,(0,255,0),2)

    cont3 = contours[2] # try third one
    rect3 = cv2.minAreaRect(cont3)
    box3 = cv2.cv.BoxPoints(rect3)
    box3 = np.int0(box3)
    cv2.drawContours(cv_image,[box3],0,(255,0,0),2)

    #TODO: 
    # once we finalize the height above the squares, I can test for size of rect, to make sure I'm not detecting a tiny square
    
    # Get center of square
    center_x, center_y = square_center_calc(box)
    cv2.circle(cv_image, (center_x, center_y), 5, (0, 255, 255), -1 )

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

