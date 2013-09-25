#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('hovercraft')
from hovercraft.msg import Thruster,Gyro

#Set DEBUG to True if you want to display the variables in console
#Not logged
DEBUG=True

#Use printD to enable DEBUG toggling functionality
def printD(string):
        global DEBUG
        if DEBUG:
                print string

def listener():
	printD("Gyro Limiter spinning")
	rospy.init_node('GyroLimiter')
	rospy.Subscriber("/thrusterProcess",Thruster, callback)
	rospy.spin()

def callback(thrusterData):
	global DEBUG
	thrust = Thruster()
	thrust.lift = thrusterData.lift
	thrust.thruster1 = thrusterData.thruster1
	thrust.thruster2 = thrusterData.thruster2
	thrust.thruster3 = thrusterData.thruster3
	thrust.thruster4 = thrusterData.thruster4
	thrust.thruster5 = thrusterData.thruster5




if __name__ == '__main__':
	listener()
