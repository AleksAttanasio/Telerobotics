#!/usr/bin/env python

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Image
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

#import service library
from std_srvs.srv import Empty

lower_green = np.array([0,40,0])
upper_green = np.array([30,220,30])

class imageGrabber:

  ##Init function, create subscriber and required vars.
  def __init__(self):
    image_sub = rospy.Subscriber("/g500/camera1",Image,self.image_callback)
    self.bridge = CvBridge()
    self.height=-1
    self.width=-1
    self.channels=-1

  ##Image received -> process the image
  def image_callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    self.height, self.width, self.channels = cv_image.shape

    bin_image = cv2.inRange(cv_image, lower_green, upper_green)
    edges = cv2.Canny(bin_image, 10, 250, apertureSize = 3)
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, 60, 20)
    m = np.array([])
    if (lines!=None): 
	for x1, y1, x2, y2 in lines[0]: 
		cv2.line(cv_image,(x1,y1),(x2,y2),(0,255,0),2)
		current_m = ((y2 - y1)/(x2 - x1))
		m = np.append(m, current_m)
    m_coeff_mean = np.mean(m)
    print m_coeff_mean
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

  ##Return size of the image
  def getSize(self):
    return self.width,self.height

if __name__ == '__main__':
  #topic to command
  twist_topic="/g500/velocityCommand"
  #base velocity for the teleoperation (0.2 m/s) / (0.2rad/s)
  baseVelocity=0.5

  ##create the publisher
  rospy.init_node('pipeFollowing')
  pub = rospy.Publisher(twist_topic, TwistStamped,queue_size=1)
  
  ##wait for benchmark init service
  rospy.wait_for_service('/startBench')
  start=rospy.ServiceProxy('/startBench', Empty)
  
  ##wait for benchmark stop service
  rospy.wait_for_service('/stopBench')
  stop=rospy.ServiceProxy('/stopBench', Empty)

  #Create the imageGrabber
  IG=imageGrabber()

  start()
  while not rospy.is_shutdown():
    msg = TwistStamped()

    #get width x height of the last received image
    imwidth,imheight=IG.getSize()

   
    msg.twist.linear.x=0
    msg.twist.linear.y=0
    msg.twist.linear.z=0
    msg.twist.angular.z=0.0

    pub.publish(msg)
    
    rospy.sleep(0.1)
  
  stop()

