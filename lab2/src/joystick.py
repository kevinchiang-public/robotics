#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('hovercraft')
roslib.load_manifest('lab2')
from sensor_msgs.msg import Joy
from lab2.msg import Movement
from lab2.msg import Switcher
import math

'''
Notes:

Start button logic might need some comments

Python Ternary Operator
variable = (True) if (Condition) else (False)

C-style Ternary operator
variable = (Condition)?(True):(False)
'''

#Set DEBUG to True if you want to display the variables in console
#Not logged
DEBUG=True

#Use printD to enable DEBUG toggling functionality
def printD(string):
        global DEBUG
        if DEBUG:
                print string

def callback(joyData):
	arbitratorPub = rospy.Publisher('/joyArbitrator/',)

def listener():
	rospy.init_node('joystick')
	rospy.Subscriber('/joy',Joy, callback)

class XboxTeleop():
        thrusterPub = rospy.Publisher('/joyOut',Movement)
        liftOn = False
        startButtonDepressed = False

if __name__ == '__main__':
	listener()


