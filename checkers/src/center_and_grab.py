#!/usr/bin/env python

import rospy
import baxter_interface
import numpy as np
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# HSV thresholding limits for RED
low_red = np.array([0,50,50])
high_red = np.array([10,255,255])

# This function takes a ros image, converts it to an openCV image, then thresholds it 
def callback(ros_img):

    # Create "bridge" between ros and opencv and convert image
    bridge = CvBridge()
    try:
      cv_image = bridge.imgmsg_to_cv2(ros_img, "bgr8") # (color image with blue-green-red color order)

    except CvBridgeError as e:
      print(e)	
    height, width, depth = cv_image.shape

    # Convert image to HSV format then to binary mask
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    binary_mask = cv2.inRange(hsv_image, low_red, high_red) # returns a binary mask, where white px: in range, black: not in range 

    # Threshold and find edges of red piece
    retVal,thresholded_img = cv2.threshold(binary_mask,127,255,0)
    contours, hierarchy = cv2.findContours(thresholded_img,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
    # contours is a Python list of all the contours in the image. Each individual contour is a Numpy array of (x,y) coordinates   
    # of boundary points of the object.

    # View images on screen
    cv2.imshow("Original Image", cv_image)
    cv2.imshow("Thresholded Image", thresholded_img)
    cv2.waitKey(3)

# Listens to the left hand camera
def listener():

    # Creates windows for openCV images
    cv2.namedWindow("Original Image", 1)
    cv2.namedWindow("Thresholded Image", 2)

    # Initiate node for left hand camera
    rospy.init_node('left_hand_camera', anonymous=True)

    # Create subscriber to grab image from camera feed 
    rospy.Subscriber("/cameras/left_hand_camera/image", Image, callback)

    # Don't exit
    rospy.spin()

if __name__ == '__main__':
     listener()

