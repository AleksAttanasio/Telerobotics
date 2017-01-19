#!/usr/bin/env python

from geometry_msgs.msg import TwistStamped
import rospy
import numpy as np
import tf
#import service library
from std_srvs.srv import Empty
from std_msgs.msg import Float64MultiArray

### WAYPOINTS
## rename to waypoints in order to use them
#waypointsTurns=[[-1.4,-4.5,7.5],[1.49,4.8,7.5],[7.4,2.975,7.5],[9.29,9.0,7.5],[7.15,9.68,7.5]]
#waypointsHeights=[[-4.96,-15.9,6.92],[-2.16,-6.9,6.92],[-1.96,-6.3,7.5],[1.4,4.5,7.5],[1.6,5.15,6.97],[3.84,12.35,6.97],[4.07,13,6.43],[4.63,14.8,6.43],[4.85,15.43,5.9],[6.53,20.83,5.9]]
#waypointsBasic=[[1.4,4.5,7.5],[-1.4,-4.5,7.5]]
waypoints=[[-1.4,-4.5,7.5],[1.49,4.8,7.5],[7.4,2.975,7.5],[9.29,9.0,7.5],[7.15,9.68,7.5]]

#topic to command
twist_topic="/g500/thrusters_input"
#base velocity for the teleoperation (0.5 m/s) / (0.5rad/s)
baseVelocity=0.5
gain = 0.07
threshold = 0.25

##create the publisher
rospy.init_node('waypointFollow')
pub = rospy.Publisher(twist_topic, Float64MultiArray ,queue_size=1)

##wait for benchmark init service
rospy.wait_for_service('/startBench')
start=rospy.ServiceProxy('/startBench', Empty)

##wait for benchmark stop service
rospy.wait_for_service('/stopBench')
stop=rospy.ServiceProxy('/stopBench', Empty)

#where are we moving to
currentwaypoint=1

listener = tf.TransformListener()

start()

while not rospy.is_shutdown() and currentwaypoint < len(waypoints):
	
  try:
  	(trans,rot) = listener.lookupTransform("/world","/girona500", rospy.Time(0))

  except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
	continue

  wRv = tf.transformations.quaternion_matrix(rot)
  wTv = tf.transformations.translation_matrix(trans)
  #print waypoints[currentwaypoint]
  wMv = np.dot(wTv, wRv)
  wMp = tf.transformations.translation_matrix(waypoints[currentwaypoint])
  vMw = tf.transformations.inverse_matrix(wMv)
  vMp = np.dot(vMw, wMp)
  # vMp = vMw . Wmp = inverse(wMv). Wmp
  vTp = tf.transformations.translation_from_matrix(vMp)

  print vTp

  msg = Float64MultiArray()
  msg.data = [0, 0, 0, 0, 0]

  xErr = vTp[0]
  yErr = vTp[1]
  zErr = vTp[2]
  x_basicVelocity = 0.5 * gain * xErr
  y_basicVelocity = 1.05 * gain * yErr
  if(zErr < 0 and abs(zErr) <= 0.30):
	z_basicVelocity = 6 * gain * zErr
  if(zErr < 0 and abs(zErr) > 0.30):
	z_basicVelocity = 1.5 * gain * zErr
  if(zErr > 0):
	z_basicVelocity = 1.05 * gain * zErr
  
  msg.data = [-x_basicVelocity, -x_basicVelocity, -z_basicVelocity, -z_basicVelocity, y_basicVelocity]
  #msg.data = [0, 0, 0, 0, y_basicVelocity]
  #msg.data = [0, 0, -z_basicVelocity, -z_basicVelocity, 0]

  #msg.twist.linear.x=0.0
  #msg.twist.linear.y=0.0
  #msg.twist.linear.z=0.0
  #msg.twist.angular.z=0.0
  pub.publish(msg)

  if(abs(xErr) < threshold and abs(yErr) < threshold and abs(zErr) < threshold):
	print "Arrived to waypoint number: {0}/{1}".format(currentwaypoint, len(waypoints) - 1)
	currentwaypoint = currentwaypoint + 1
	if(currentwaypoint+1 == len(waypoints)):
		print "==== END OF TRACK ===="
		
  
  rospy.sleep(0.1)

stop()
