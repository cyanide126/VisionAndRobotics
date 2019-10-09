#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
from matplotlib import pyplot as plt
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send messages to a topic named image_topic
    self.image_pub = rospy.Publisher("image_topic",Image, queue_size = 1)
    # initialize a publisher to send joints' angular position to a topic called joints_pos
    self.joints_pub = rospy.Publisher("joints_pos",Float64MultiArray, queue_size=10)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub = rospy.Subscriber("/robot/camera1/image_raw",Image,self.callback)

  # Recieve data, process it, and publish
  def callback(self,data):
    # Recieve the image
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
      
      
    # loading template for links as binary image (used in lab 2)
  #  self.link1 = cv2.inRange(cv2.imread('link1.png', 1), (200, 200, 200), (255, 255, 255))
  #  self.link2 = cv2.inRange(cv2.imread('link2.png', 1), (200, 200, 200), (255, 255, 255))
  #  self.link3 = cv2.inRange(cv2.imread('link3.png', 1), (200, 200, 200), (255, 255, 255))   
    
    # Perform image processing task (your code goes here)
    
    """
    hist = cv2.calcHist([cv_image[:,:,1]],[0],None,[256],[0,256])
    plt.hist(cv_image.ravel(),256,[0,256])
    plt.show()
    plt.xlabel("Pixel Value")
    
    
    for i in range(len(cv_image)):
	for j in range(len(cv_image[i])):
            if(((cv_image[i][j][0] >=51) and (cv_image[i][j][0] <=255))and((cv_image[i][j][1] >=0) and (cv_image[i][j][1] <=153)) and ((cv_image[i][j][2] >=0) and (cv_image[i][j][2] <=153))):
                cv_image[i][j]=255
            else:
                cv_image[i][j]=0
    """
    ret,red1 = cv2.threshold(cv_image[:,:,0],0,255,cv2.THRESH_BINARY)
    ret,red2 = cv2.threshold(cv_image[:,:,1],0,10,cv2.THRESH_BINARY_INV)
    ret,red3 = cv2.threshold(cv_image[:,:,2],0,100,cv2.THRESH_BINARY_INV)
    cv2.imshow('image_window0',red1)
    cv2.imshow('image_window1',red2)
    cv2.imshow('image_windo2',red3)
    cv2.imshow('image_window3',red3*red2*red1)
    ##cv2.imshow('image_window1',cv_image[:,:,1])
    ##cv2.imshow('image_window2',cv_image[:,:,2])    

    # The image is loaded as cv_imag

    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)

    cv2.imshow('window', cv_image)
    cv2.waitKey(3)

    # change te value of self.joint.data to your estimated value from thew images once you have finalized the code
    self.joints = Float64MultiArray()
    self.joints.data = np.array([0, 0, 0])

    # Publish the results - the images are published under a topic named "image_topic" and calculated joints angles are published under a topic named "joints_pos"
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
      self.joints_pub.publish(self.joints)
    except CvBridgeError as e:
      print(e)

# call the class
def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)


