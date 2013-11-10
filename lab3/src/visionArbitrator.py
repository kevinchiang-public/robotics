#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('lab3')

from sensor_msgs.msg import Joy
from hovercraft.msg import Thruster
from lab3.msg import Switcher, MovementRaw, Movement
from copy import deepcopy as deep
class VisionArbitrator():
    def __init__(self):
        #Idea: intercept topic that typically publishes to ball detector/landmark using remapping in the
        #launch file.  Then publish out to one or the other pending on the switch state from the
        #switch node.

if __name__ == '__main__':
    rospy.init_node('VisionArbitrator')
    try:
        ne = VisionArbitrator()
        rospy.spin()
    except rospy.ROSInterruptException: pass
