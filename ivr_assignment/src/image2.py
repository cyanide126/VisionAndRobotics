#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
import math
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError
from scipy.optimize import fsolve


class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image2', anonymous=True)
    # initialize a publisher to send images from camera2 to a topic named image_topic2
    self.bridge = CvBridge()
    #scale (projection in plane parallel to camera through yellow blob) determined for all angles=0
    self.image_pub2 = rospy.Publisher("image_topic2",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.callback2)
    self.image1_sub = rospy.Subscriber("/camera1/blob_pos",Float64MultiArray,self.callbackmaster)
    # initialize a publisher to publish position of blobs
    self.blob_pub2 = rospy.Publisher("/camera2/blob_pos",Float64MultiArray, queue_size=10)
    # initialize a publisher to send joints' angular position to the robot
    self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
    self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
    self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
    self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
    # initialize the bridge between openCV and ROS
    

  #___________________detection of the blobs__________________________
  def detect_red(self,image):
      # Isolate the blue colour in the image as a binary image
      mask = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
      # This applies a dilate that makes the binary region larger (the more iterations the larger it becomes)
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      # Obtain the moments of the binary image
      M = cv2.moments(mask)
      # Calculate pixel coordinates for the centre of the blob
      if M['m00'] == 0:
	return np.array([np.nan,np.nan])
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])

  # Detecting the centre of the green circle
  def detect_green(self,image):
      mask = cv2.inRange(image, (0, 100, 0), (0, 255, 0))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      if M['m00'] == 0:
	return np.array([np.nan,np.nan])
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])

  # Detecting the centre of the blue circle
  def detect_blue(self,image):
      mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      if M['m00'] == 0:
	return np.array([np.nan,np.nan])
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])

  # Detecting the centre of the yellow circle
  def detect_yellow(self,image):
      mask = cv2.inRange(image, (0, 100, 100), (0, 255, 255))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      if M['m00'] == 0:
	return np.array([np.nan,np.nan])
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])

  #______________get the projection from published blob data______________
  def eliminate_nonvisible_blobs(self,blobs):    
    if np.isnan(blobs[4]):
      blobs[4]=blobs[2]
      blobs[5]=blobs[3]
    if np.isnan(blobs[6]):
      blobs[6]=blobs[4]
      blobs[7]=blobs[5]
    return blobs

  def get_projection(self,blobs,r_yellow,scale):
    corrected_blobs = self.eliminate_nonvisible_blobs(blobs)
    pos_cam = np.array(corrected_blobs).reshape((4,2))
    pos_cam = scale*pos_cam
    pos_cam[0,1] = pos_cam[0,1]+r_yellow #takes into account that only half yellow blob is visible
    pos_cam = pos_cam-pos_cam[0]
    pos_cam[:,1]=-pos_cam[:,1]
    return np.array(pos_cam)

  #________________use projection to get blob position___________________
  
  #perform a weighted average over the two z-measurements
  #blobs closer to the camera are more distorted than blobs farer away. Use the z-value from blob which is farer away
  def z_average(self,z1,z2,x,y):
    w1 = (5-x)**2
    w2 = (5+y)**2
    if x==-y:
      return (z1+z2)/2
    else:
      return (w1*z1+w2*z2)/(w1+w2)
  
  def yellow_blob_measured(self,cam1,cam2):
    return np.array([cam2[0,0],cam1[0,0],self.z_average(cam1[0,1],cam2[0,1],cam2[0,0],cam1[0,0])])
  def blue_blob_measured(self,cam1,cam2):
    return np.array([cam2[1,0],cam1[1,0],self.z_average(cam1[1,1],cam2[1,1],cam2[1,0],cam1[1,0])])
  def green_blob_measured(self,cam1,cam2):
    return np.array([cam2[2,0],cam1[2,0],self.z_average(cam1[2,1],cam2[2,1],cam2[2,0],cam1[2,0])])
  def red_blob_measured(self,cam1,cam2):
    return np.array([cam2[3,0],cam1[3,0],self.z_average(cam1[3,1],cam2[3,1],cam2[3,0],cam1[3,0])])
  def blobs_measured(self,cam1,cam2):
    return np.array([self.yellow_blob_measured(cam1,cam2),self.blue_blob_measured(cam1,cam2),
		     self.green_blob_measured(cam1,cam2),self.red_blob_measured(cam1,cam2)])
    

  #__________________matrix calculation for green blob __________________
  #position of green blob is x,y,z without rotation around z axis (yellow blob) and then
  #rotated by a rotation matrix around z: rot_z(theta1)*xyz(theta2,theta3)
  def pos_green_blob(self,theta1,theta2,theta3):
    x = 3*np.sin(theta3)
    y = -3*np.sin(theta2)*np.cos(theta3)
    z = 2+3*np.cos(theta2)*np.cos(theta3)
    rot = np.array([[np.cos(theta1),-np.sin(theta1),0],
    		[np.sin(theta1),np.cos(theta1),0],
    		[0,0,1]])

    return rot.dot(np.array([x,y,z]))

  #_____________rotation-matrix for red blob______________________
  def rotz(self,theta):
    return np.array([[np.cos(theta),-np.sin(theta),0],
	  	   [np.sin(theta),np.cos(theta),0],
	  	   [0,0,1]])
  def rotx(self,theta):
    return np.array([[1,0,0],
	  	   [0,np.cos(theta),-np.sin(theta)],
	  	   [0,np.sin(theta),np.cos(theta)]])
  def roty(self,theta):
    return np.array([[np.cos(theta),0,-np.sin(theta)],
	  	   [0,1,0],
	  	   [np.sin(theta),0,np.cos(theta)]])
  def rot_tot(self,theta1,theta2,theta3,theta4):
    return self.rotz(theta1).dot(self.rotx(theta2).dot(self.roty(-theta3).dot(self.rotx(theta4))))
  
  def pos_red_blob(self,green_blob,theta1,theta2,theta3,theta4):
    return green_blob+2*self.rot_tot(theta1,theta2,theta3,theta4).dot(np.array([0,0,1]))

  #________________Target Detection______________________________________________

  def detectTarget(self):
    """
    self.getbox()
    self.threshTarget()
    self.cropSearch(self.threshedTarget)
    self.cropImage(0,0)
    #self.cropPic()
    self.minimizeError()
    self.position()
    self.targetCenter()
    self.threshBase()
    
    self.detectOrange()
    self.contours()
    self.findSquare()
    self.drawFinal()
    """
    self.getSphere(self.cv_image2)
    print(self.findCenter(self.sphere))

  #use color threshhold to segment target
  def detectOrange(self, sourceImg):
    targetImg = cv2.inRange(sourceImg,(10,10,120),(100,255,255))
    return targetImg

  #get contours of the orange objects
  def contours(self, sourceImg):
    targetImg = self.detectOrange(sourceImg)
    img = cv2.bitwise_not(targetImg)
    contours, hierarchy = cv2.findContours(img, 1, 2)

    self.cnt1 = contours[0]
    self.cnt2 = contours[1]


  #calculate the circularity of each contour
  def getCircularity(self,cnt):
    perimeter = cv2.arcLength(cnt, True)
    area = cv2.contourArea(cnt)
    
    return (4*math.pi*area)/(perimeter**2)

  #compares two contour and returns the one that is less circular
  def compareCnts(self):
    if self.getCircularity(self.cnt1)>self.getCircularity(self.cnt2):
      return self.cnt1
    else:
      return self.cnt2

  #display the contour of a sphere
  def getSphere(self, sourceImg):
    self.contours(sourceImg)
 
    self.sphere = cv2.inRange(sourceImg,(100,100,100),(101,101,101))
    cv2.drawContours(self.sphere, [self.compareCnts()], 0, (255,0,255), 1)

    cv2.imshow("box",self.sphere)

  #findCenter of the contour
  def findCenter(self, cnt):
    M = cv2.moments(cnt)
    
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
   
    return [cx,cy]

  """
  def threshTarget(self):

    ret,img1 = cv2.threshold(self.cv_image2[:,:,0],100,255,cv2.THRESH_BINARY_INV)
    ret,img2 = cv2.threshold(self.cv_image2[:,:,1],150,255,cv2.THRESH_BINARY_INV)
    #ret,img3 = cv2.threshold(self.cv_image2[:,:,2],150,255,cv2.THRESH_BINARY_INV)
    ret,img4 = cv2.threshold(self.cv_image2[:,:,2],120,255,cv2.THRESH_BINARY)

    cv2.imshow('image_window3',img4*img2*img1)
    self.threshedTarget=img4*img2*img1

    return img4*img2*img1

  def getbox(self):
    self.box=cv2.inRange(cv2.imread('box.png',1), (200, 200, 200), (255,255,255))
    
  def cropPic(self):
    self.pic=cv2.inRange(cv2.imread('pic.png',1), (200, 200, 200), (255,255,255))
    cv2.imshow('pic',self.pic)
  
  #initial crop to narrow down search area
  def cropSearch(self, img):
    #take lower quadrant
    rows, cols = img.shape
    cropped = img[rows/2:(rows/4)*3-80, cols/2:cols-80]
    cv2.imshow('cropped', cropped)
    self.searchArea=cropped

  def distanceTransform(self,img):
    img_inv =cv2.bitwise_not(img)
    return cv2.distanceTransform(img_inv, cv2.DIST_L1,5)
  
  def cropImage(self,rowShift,colShift):
    rows, cols = self.box.shape
    cropped = self.searchArea[rowShift:rowShift+rows,colShift:colShift+cols]
    cv2.imshow('box', self.box)
    cv2.imshow('boxcrop', cropped)
    return cropped

  def minimizeError(self):
    minDst=1000000**1000000
    rowsArea, colsArea = self.searchArea.shape
    self.rowsBox, self.colsBox = self.box.shape
    for r in range(rowsArea/self.rowsBox):
       for c in range(colsArea/self.colsBox):
         dst=self.box*self.distanceTransform(self.cropImage(r*self.rowsBox,c*self.colsBox))
         totalDst=0
         rows,cols= dst.shape
         for p in range(rows):
           for q in range(cols):
             totalDst+=dst[p][q]
         if totalDst<minDst:
           minDst=totalDst
           self.rShift=r
           self.cShift=c
    cv2.imshow('FINAL',self.cropImage(self.rShift*self.rowsBox,self.cShift*self.colsBox))
    print("XO",rShift,cShift,colsBox)
    

  def position(self):
    rows,cols=self.threshTarget.shape
    self.rowsDst=(rows/2)+self.rShift*self.rowsBox
    self.colsDst=(cols/2)+self.cShift*self.colsBox

  def targetCenter(self):
    self.yCent=self.rowsDst+(self.rowsBox/2)
    self.xCent=self.colsDst+(self.colsBox/2)

  def threshBase(self):
    ret,img1 = cv2.threshold(self.cv_image2[:,:,0],10,255,cv2.THRESH_BINARY_INV)
    ret,img2 = cv2.threshold(self.cv_image2[:,:,1],10,255,cv2.THRESH_BINARY_INV)
    ret,img3 = cv2.threshold(self.cv_image2[:,:,2],10,255,cv2.THRESH_BINARY_INV)
   
    cv2.imshow('base',img3*img2*img1)
    self.threshedBase=img3*img2*img1
  """
  

  # _______________Recieve data, process it, and publish______________________
  def callback2(self,data):
    # Recieve the image
    try:
      self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    self.blob_pos2=Float64MultiArray()
    self.blob_pos2.data=np.array([self.detect_yellow(self.cv_image2),self.detect_blue(self.cv_image2),self.detect_green(self.cv_image2),self.detect_red(self.cv_image2)]).flatten()
    im2=cv2.imshow('window2', self.cv_image2)
    cv2.waitKey(1)

    # Publish the results
    try: 
      self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
      self.blob_pub2.publish(self.blob_pos2)
    except CvBridgeError as e:
      print(e)
    
    self.detectTarget()
    



  #_________________________combine both images____________________________
  def callbackmaster(self,data):
    #save the projection into a matrix
    blob_pos1 = np.array(data.data)
    pos_cam1 = self.get_projection(blob_pos1,0.43,5/134.)
    pos_cam2 = self.get_projection(self.blob_pos2.data,0.3,5/132.)
    x_measured = self.blobs_measured(pos_cam1,pos_cam2)

    x_measured_blue = x_measured[1]
    x_measured_green = x_measured[2]
    x_measured_red = x_measured[3]
    #x_diff_red_green = x_measured_red-x_measured_green

    #define the function for fsolve (numerical solver for the angles given the measured position of the green blob)
    #def function_for_fsolve_green(theta):
	#return np.array([3*(np.cos(theta[0])*np.sin(theta[2])+np.sin(theta[0])*np.sin(theta[1])*np.cos(theta[2]))-x_measured_green[0],
	#		3*(np.sin(theta[0])*np.sin(theta[2])-np.cos(theta[0])*np.sin(theta[1])*np.cos(theta[2]))-x_measured_green[1],
	#		2+3*np.cos(theta[1])*np.cos(theta[2])-x_measured_green[2]])
    #perform solver
    #theta_est = fsolve(function_for_fsolve_green,np.array([0,0,0]),xtol=1e-8)


    #def function_for_fsolve(theta):
    #    temp = self.roty(theta[2]).dot(self.rotx(-theta[1]).dot(self.rotz(-theta[0]).dot(x_diff_red_green/2)))
    #    return np.array([3*(np.cos(theta[0])*np.sin(theta[2])+np.sin(theta[0])*np.sin(theta[1])*np.cos(theta[2]))-x_measured_green[0],
	#		3*(np.sin(theta[0])*np.sin(theta[2])-np.cos(theta[0])*np.sin(theta[1])*np.cos(theta[2]))-x_measured_green[1],
	#		2+3*np.cos(theta[1])*np.cos(theta[2])-x_measured_green[2],
	#		np.arctan(-temp[1]/temp[2])-theta[3]])
    #theta_est = fsolve(function_for_fsolve,np.array([0,0,0,0]),xtol=1e-8)




    #define desired joint angles
    q_d = [0,np.pi/4,np.pi/4,0]		#move robot here
    self.joint1=Float64()
    self.joint1.data= q_d[0]
    self.joint2=Float64()
    self.joint2.data= q_d[1]
    self.joint3=Float64()
    self.joint3.data= q_d[2]
    self.joint4=Float64()
    self.joint4.data= q_d[3]

    #theoretical positions out of desired joint angles
    theoretical_pos_red = self.pos_red_blob(self.pos_green_blob(q_d[0],q_d[1],q_d[2]),*q_d)
    theoretical_pos_green = self.pos_green_blob(q_d[0],q_d[1],q_d[2])

    #use fsolve to get the estimated joint angles
    '''def function_for_fsolve(theta):
      link3 = self.pos_red_blob(0,theta[0],theta[1],theta[2],theta[3])
      x_green = 3*(np.cos(theta[0])*np.sin(theta[2])+np.sin(theta[0])*np.sin(theta[1])*np.cos(theta[2]))
      y_green = 3*(np.sin(theta[0])*np.sin(theta[2])-np.cos(theta[0])*np.sin(theta[1])*np.cos(theta[2]))
      z_green = 2+3*np.cos(theta[1])*np.cos(theta[2])
      x_red = x_green + link3[0]
      y_red = y_green + link3[1]
      z_red = z_green + link3[2]
      return np.array([x_green-x_measured_green[0],
	               y_green-x_measured_green[1],
		       z_green-x_measured_green[2],
		       #x_red-x_measured_red[0],
		       #y_red-x_measured_red[1]])
		       z_red-x_measured_red[2]])
    theta_est = fsolve(function_for_fsolve,np.array([0,0,0]),xtol=1e-10)
    theta_est[0] = theta_est[0] % 2*np.pi
    if theta_est[0]<np.pi:
      theta_est[0]=theta_est[0]+2*np.pi
    if theta_est[0]>np.pi:
      theta_est[0]=theta_est[0]-2*np.pi
    #another approach
    def function_for_fsolve23(theta):
      x_green = 3*(np.cos(theta[0])*np.sin(theta[2])+np.sin(theta[0])*np.sin(theta[1])*np.cos(theta[2]))
      y_green = 3*(np.sin(theta[0])*np.sin(theta[2])-np.cos(theta[0])*np.sin(theta[1])*np.cos(theta[2]))
      z_green = 2+3*np.cos(theta[1])*np.cos(theta[2])
      return np.array([x_green-x_measured_green[0],
	               y_green-x_measured_green[1],
		       z_green-x_measured_green[2]])
    theta_est23 = fsolve(function_for_fsolve23,np.array([0,0,0]))
    
    def function_for_fsolve14(theta):
      link3 = self.pos_red_blob(0,theta[0],theta_est23[1],theta[2],theta[1])
      x_green = 3*(np.cos(theta[0])*np.sin(theta[2])+np.sin(theta[0])*np.sin(theta_est23[1])*np.cos(theta[2]))
      y_green = 3*(np.sin(theta[0])*np.sin(theta[2])-np.cos(theta[0])*np.sin(theta_est23[1])*np.cos(theta[2]))
      z_green = 2+3*np.cos(theta_est23[1])*np.cos(theta[2])
      x_red = x_green + link3[0]
      y_red = y_green + link3[1]
      z_red = z_green + link3[2]
      return np.array([x_red-x_measured_red[0],
		       y_red-x_measured_red[1],
		       z_red-x_measured_red[2]])
    theta_est14 = fsolve(function_for_fsolve14,np.array([theta_est23[0],0,theta_est23[2]]))
    theta_est = np.array([theta_est14[0],theta_est23[1],theta_est23[2],theta_est14[1]])
    theta_est[0] = theta_est[0] % 2*np.pi
    if theta_est[0]<np.pi:
      theta_est[0]=theta_est[0]+2*np.pi
    if theta_est[0]>np.pi:
      theta_est[0]=theta_est[0]-2*np.pi'''
    def get_vector(theta1,theta2,theta3):
      s1 = np.sin(theta1)
      c1 = np.cos(theta1)
      s2 = np.sin(theta2)
      c2 = np.cos(theta2)
      s3 = np.sin(theta3)
      c3 = np.cos(theta3)
      mat = np.array([[c1*c3-s1*s2*s3,-s1*c2,+c1*s3+s1*s2*c3],
		      [s1*c3+c1*s2*s3,c1*c2,+s1*s3-c1*s2*c3],
		      [-c2*s3,s2,c2*c3]])
      z = np.array([0,0,1])
      z = mat.dot(z)*3
      return z
    def get_theta(x_measured):
      a = np.array([0,0,1])
      b = x_measured[2]-x_measured[1]
      b = b/np.linalg.norm(b)
      v = np.cross(a,b)
      s = np.linalg.norm(v)
      c = a.dot(b)
      vx = np.array([[0,-v[2],v[1]],
		    [v[2],0,-v[0]],
		    [-v[1],v[0],0]])
      one = np.array([[1,0,0],[0,1,0],[0,0,1]])
      mat = one+vx+vx.dot(vx)*(1-c)/s**2
      print(mat)

      theta1 = np.arctan2(-mat[0,1],mat[1,1])
      theta2 = np.arctan2(mat[2,1],np.sqrt(mat[2,0]**2+mat[2,2]**2))
      theta3 = np.arctan2(-mat[2,0],mat[2,2])
      theta = np.array([theta1,theta2,theta3])

      d = x_measured[3]-x_measured[2]
      dd = self.roty(theta[2]).dot(self.rotx(-theta[1]).dot(self.rotz(-theta[0]).dot(d)))
      theta4 = np.arctan2(-dd[1],dd[2])
      theta = np.append(theta,theta4)
      return theta
    print(get_theta(x_measured))
 


    

    #test
    '''meas_angle_pos_red = self.pos_red_blob(self.pos_green_blob(theta_est[0],theta_est[1],theta_est[2]),*theta_est)
    print(function_for_fsolve23(theta_est23))
    print(function_for_fsolve14(theta_est14))
    print("pos out of measured angle:\t{}".format(meas_angle_pos_red))
    print("measured pos:\t\t\t{}".format(x_measured_red))
    print("theoretical position:\t\t{}\n".format(theoretical_pos_red))
    print("measured angle:\t\t\t{}".format(theta_est))
    print("desired angle:\t\t\t{}\n\n".format(q_d))'''
    
    #publish results
    try: 
      self.robot_joint1_pub.publish(self.joint1)
      self.robot_joint2_pub.publish(self.joint2)
      self.robot_joint3_pub.publish(self.joint3)
      self.robot_joint4_pub.publish(self.joint4)
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
