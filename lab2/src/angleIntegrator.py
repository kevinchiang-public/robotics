#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('lab2')
from sensor_msgs.msg import Joy
from lab2.msg import Movement

def listener():
	rospy.init_node('Joystick Angle Integrator')
	rospy.Subscriber('/joyOut', Joy, callback)
	rospy.spin()

def callback(joyData):
	

if __name__ == '__main__':
        listener() 
