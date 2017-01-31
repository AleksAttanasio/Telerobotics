#!/usr/bin/env python

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Image
from sympy import *
from sympy.geometry import *
from sympy.plotting import plot
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
    
    #center = Point(160,120)
    #r1 = 50
    #r2 = 100
    #c1 = Circle(center, r1)
    #c2 = Circle(center, r2)
    begin_x = 50
    end_x = 100
    lower_y = 60
    upper_y = 120
    detect_line1 = Line(Point(begin_x,lower_y),Point(end_x,lower_y))
    detect_line2 = Line(Point(begin_x,upper_y),Point(end_x,upper_y))
    #detect_line1 = Circle(Point(160,120),80)
    #detect_line2 = Circle(Point(160,120),100)
    bin_image = cv2.inRange(cv_image, lower_green, upper_green)
    edges = cv2.Canny(bin_image, 10, 250, apertureSize = 3)
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, 60, 20)
    
    if (lines!=None): 
	for x1, y1, x2, y2 in lines[0]: 
		cv2.line(cv_image,(x1,y1),(x2,y2),(0,255,0),2)
		segment = Segment(Point(x1,y1), Point(x2,y2))
		intersect1 = intersection(segment, detect_line1)
		intersect2 = intersection(segment, detect_line2)
		if(intersect1 != None):
			cv2.circle(cv_image,(intersect1[0].x,intersect1[0].y), 1, (0,0,255), -1)
		if(intersect2 != None):
			cv2.circle(cv_image,(intersect2[0].x,intersect2[0].y), 1, (0,0,255), -1)
		#print intersect
		#print len(intersect)

		#cv2.circle(cv_image,(x1,y1), 1, (0,0,255), -1)
		#cv2.circle(cv_image,(x2,y2), 1, (0,0,255), -1)
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

