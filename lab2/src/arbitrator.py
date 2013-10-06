#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('lab2')
from sensor_msgs.msg import Joy
from lab2.msg import Switcher

def callback(switcher):


def listener():
	rospy.init_node('arbitrator')
	rospy.Subscriber('/joyArbitrator', Switcher, callback)

if __name__ == '__main__':
	listener()
