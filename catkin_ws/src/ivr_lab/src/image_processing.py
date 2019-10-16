#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
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
    self.link1 = cv2.inRange(cv2.imread('link1.png', 1), (200, 200, 200), (255, 255, 255))
    self.link2 = cv2.inRange(cv2.imread('link2.png', 1), (200, 200, 200), (255, 255, 255))
    self.link3 = cv2.inRange(cv2.imread('link3.png', 1), (200, 200, 200), (255, 255, 255))   
    
    
    # Perform image processing task (your code goes here)
    def thresBlue():

    	ret,red1 = cv2.threshold(cv_image[:,:,0],10,255,cv2.THRESH_BINARY)
    	ret,red2 = cv2.threshold(cv_image[:,:,1],50,255,cv2.THRESH_BINARY_INV)
    	ret,red3 = cv2.threshold(cv_image[:,:,2],50,255,cv2.THRESH_BINARY_INV)

    	#cv2.imshow('image_window3',red3*red2*red1)
	return red3*red2*red1
    def thresGreen():
        ret,red1 = cv2.threshold(cv_image[:,:,0],50,255,cv2.THRESH_BINARY_INV)
    	ret,red2 = cv2.threshold(cv_image[:,:,1],10,255,cv2.THRESH_BINARY)
    	ret,red3 = cv2.threshold(cv_image[:,:,2],50,255,cv2.THRESH_BINARY_INV)

    	#cv2.imshow('image_window2',red3*red2*red1)
	return red3*red2*red1
    def thresRed():
        ret,red1 = cv2.threshold(cv_image[:,:,0],50,255,cv2.THRESH_BINARY_INV)
    	ret,red2 = cv2.threshold(cv_image[:,:,1],50,255,cv2.THRESH_BINARY_INV)
    	ret,red3 = cv2.threshold(cv_image[:,:,2],10,255,cv2.THRESH_BINARY)

    	#cv2.imshow('image_window1',red3*red2*red1)
	return red3*red2*red1

    def thresYellow():
        ret,red1 = cv2.threshold(cv_image[:,:,0],170,255,cv2.THRESH_BINARY_INV)
    	ret,red2 = cv2.threshold(cv_image[:,:,1],180,255,cv2.THRESH_BINARY)
    	ret,red3 = cv2.threshold(cv_image[:,:,2],245,255,cv2.THRESH_BINARY)

    	#cv2.imshow('image_window4',red3*red2*red1)
	return red3*red2*red1


    def findCenter(thresh):
        contours,hierarchy = cv2.findContours(thresh, 1, 2)
        cnt = contours[0]
        M = cv2.moments(cnt)

        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        center = [cx,cy]
        return center

    def pixelCenter():
        centerRed=findCenter(thresRed())
        centerGreen=findCenter(thresGreen())
        centerBlue=findCenter(thresBlue())
        centerYellow=findCenter(thresYellow())
        center = [centerRed, centerGreen, centerBlue,centerYellow]
        return center
    
    def meterCenter():
        pixCent=pixelCenter()
        

        gray_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        centerImage=findCenter(gray_img)

        #distance between blobs = 3 meters
        pixPerMeter=float((pixCent[1][1]-pixCent[2][1])/3)
  
        for i in range(len(pixCent)):
           pixCent[i][0]= (pixCent[i][0]-centerImage[0])/pixPerMeter
           pixCent[i][1]= (pixCent[i][1]-centerImage[1])/pixPerMeter
        return pixCent
        

        
        

    def thresLink():
        ret,red1 = cv2.threshold(cv_image[:,:,0],10,255,cv2.THRESH_BINARY_INV)
    	ret,red2 = cv2.threshold(cv_image[:,:,1],10,255,cv2.THRESH_BINARY_INV)
    	ret,red3 = cv2.threshold(cv_image[:,:,2],10,255,cv2.THRESH_BINARY_INV)
    	cv2.imshow('image_window4',red3*red2*red1)
        return red3*red2*red1

    def linkCenter():
        cent = pixelCenter()

        link1Cent = [((cent[1][0]-cent[0][0])/2)+cent[0][0],((cent[1][1]-cent[0][1])/2)+cent[0][1]]
        link2Cent = [((cent[2][0]-cent[1][0])/2)+cent[1][0],((cent[2][1]-cent[1][1])/2)+cent[1][1]]
        link3Cent = [((cent[3][0]-cent[2][0])/2)+cent[2][0],((cent[3][1]-cent[2][1])/2)+cent[2][1]]

        linksCent=[link1Cent,link2Cent,link3Cent]
        return linksCent
        print(self.link1.shape)

    def cropLink():
        thresh = thresLink()
        linksCent=linkCenter()
        print(linksCent)
        link1Window=self.link1.shape
        link2Window=self.link2.shape
        link3Window=self.link3.shape
        #print(linksCent[0][1]-(link1Window[0]/2))
        link1Crop = thresh[linksCent[0][1]-(link1Window[0]/2):linksCent[0][1]+(link1Window[0]/2), linksCent[0][0]-(link1Window[1]/2):linksCent[0][0]+(link1Window[1]/2)]

        link2Crop = thresh[linksCent[1][1]-(link2Window[0]/2):linksCent[1][1]+(link1Window[0]/2), linksCent[1][0]-(link1Window[1]/2):linksCent[1][0]+(link1Window[1]/2)]

        link1Crop = thresh[linksCent[0][1]-(link1Window[0]/2):linksCent[0][1]+(link1Window[0]/2), linksCent[0][0]-(link1Window[1]/2):linksCent[0][0]+(link1Window[1]/2)]
        cv2.imshow("cropped",link1Crop)
    cropLink()
        
       
    #thresLink()
    #linkCenter()

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


