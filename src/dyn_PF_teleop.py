#!/usr/bin/env python

from geometry_msgs.msg import TwistStamped
import termios, fcntl, sys, os
import rospy

#import service library
from std_srvs.srv import Empty
from std_msgs.msg import Float64MultiArray
#topic to command
twist_topic="/g500/thrusters_input"
#base velocity for the teleoperation (0.5 m/s) / (0.5rad/s)
baseVelocity=0.5

#Console input variables to teleop it from the console
fd = sys.stdin.fileno()
oldterm = termios.tcgetattr(fd)
newattr = termios.tcgetattr(fd)
newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
termios.tcsetattr(fd, termios.TCSANOW, newattr)
oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)

##create the publisher
pub = rospy.Publisher(twist_topic, Float64MultiArray ,queue_size=1)
rospy.init_node('keyboardCommand')

##wait for benchmark init service
rospy.wait_for_service('/startBench')
start=rospy.ServiceProxy('/startBench', Empty)

##wait for benchmark stop service
rospy.wait_for_service('/stopBench')
stop=rospy.ServiceProxy('/stopBench', Empty)

print "==== Teleoperating G500 ==== \nDynamic teleoperator"\
"\nKeymap:\n"\
"u\ti\to\n\t^\n\t|\nj     <- ->\tl\n\t|\n\tV\nm\t,\t.\n\n"\
"i - j - l - , control linear velocities\nu - o - m - . control angular velocities\n"\
"e \t increase vertical velocity\n"\
"d \t decrease vertical velocity\n"\
"Use w and s for increase and decrease velocity\n"

#The try is necessary for the console input!
try:
    while not rospy.is_shutdown():
	msg = Float64MultiArray()
	msg.data = [0, 0, 0, 0, 0]
        try:
            c = sys.stdin.read(1)
            ##Depending on the character set the proper speeds
	    if c=='\n':
		start()
	  	print "Benchmarking Started!"
	    elif c==' ':
		stop()
		print "Benchmark finished!"
	    elif c=='i':
		msg.data=[-baseVelocity, -baseVelocity, 0, 0, 0]
	    elif c==',':
		msg.data=[baseVelocity, baseVelocity, 0, 0, 0]
	    elif c=='l':
		msg.data=[0, 0, 0, 0, baseVelocity]
	    elif c=='j':
		msg.data=[0, 0, 0, 0, -baseVelocity]
	    elif c=='e':
		msg.data =[0, 0, baseVelocity, baseVelocity, 0]
	    elif c=='d':
		msg.data =[0, 0, -baseVelocity, -baseVelocity, 0]
	    elif c=='o':
		msg.data=[-baseVelocity,baseVelocity, 0, 0, 0]
	    elif c=='u':
		msg.data=[baseVelocity,-baseVelocity, 0, 0, 0]
	    elif c=='m':
		msg.data=[0,0,baseVelocity,-baseVelocity, 0]	    
	    elif c=='.':
		msg.data=[0,0,-baseVelocity,baseVelocity, 0]
	    elif c=='w':
		baseVelocity = baseVelocity + baseVelocity * 0.15
		print baseVelocity
	    elif c=='s':
		baseVelocity = baseVelocity - baseVelocity * 0.15
		print baseVelocity
	    else:
		print 'wrong key pressed'
	    while c!='':
	        c = sys.stdin.read(1)
        except IOError: pass

        ##publish the message
        pub.publish(msg)
	rospy.sleep(0.1)

##Other input stuff
finally:
    termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
    fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)
