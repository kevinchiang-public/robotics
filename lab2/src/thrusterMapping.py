#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('hovercraft')
roslib.load_manifest('lab2')
from lab2.msg import Movement
from hovercraft.msg import Thruster
from sensor_msgs.msg import Joy
import math

DEBUG=False
thruster=Thruster()

def listener():
		rospy.init_node('thrusterMapping')
		rospy.Subscriber("/thrusterMapping",Movement,callback)
		rospy.Subscriber("/arbitratorThruster",Thruster,arbitratorCallback)
		rospy.spin()

def arbitratorCallback(thrust):
	global thruster
	thruster = thrust

def callback(move):
	global thruster
	global DEBUG
	theta = move.theta
	x = move.x
	y = move.y
	thrust = thruster
	#print thrust

	if theta >0:
		#Turn on 4
		thrust.thruster5 = 0
		thrust.thruster4 = math.fabs(theta)
		thrust.thruster4 = thrust.thruster4 if thrust.thruster4 < .5 else .5
	elif theta<0:
		#Turn on 5 
		thrust.thruster4 = 0
		thrust.thruster5 = math.fabs(theta)
		thrust.thruster5 = thrust.thruster5 if thrust.thruster5 < .5 else .5

	pub = rospy.Publisher('/hovercraft/Thruster',Thruster)
	if DEBUG:
		print "Theta:",theta,"\tThruster 4:",thrust.thruster4,"\tThruster 5:",thrust.thruster5
	pub.publish(thrust)



#Entry point for ROS
if __name__ == '__main__':
	listener()
