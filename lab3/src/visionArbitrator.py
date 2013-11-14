#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('lab3')
roslib.load_manifest('ballDetector')

from ballDetector.msg import ballLocation
from sensor_msgs.msg import Joy, Image
from hovercraft.msg import Thruster
from lab3.msg import Switcher, MovementRaw, Movement, DetectionSwitcher
from copy import deepcopy as deep

class VisionArbitrator():
    def __init__(self):
        #Idea: intercept topic that typically publishes to ball detector/landmark using remapping in the
        #launch file.  Then publish out to one or the other pending on the switch state from the
        #switch node.
        self.publishTarget = None
        self.ballPublisher = rospy.Publisher('/visArbitrator/toBall',Image)
        self.landPublisher = rospy.Publisher('/visArbitrator/toLand',Image)
        rospy.Subscriber('image', Image, self.cameraCallback)
        rospy.Subscriber('/switcher/visArbitrator', DetectionSwitcher, self.switcherCallback)

    def cameraCallback(self, image):
        if self.publishTarget == 'ballDetector':
            self.ballPublisher.publish(image)
        elif self.publishTarget == 'landDetector':
            self.landPublisher.publish(image)
        elif self.publishTarget == 'both':
            self.landPublisher.publish(image)
            self.ballPublisher.publish(image)
    def switcherCallback(self, switcher):
        #print switcher.state
        if switcher.state == 1:
            self.publishTarget = 'landDetector'
        elif switcher.state == 2:
            self.publishTarget = 'ballDetector'
        elif switcher.state == 3:
            self.publishTarget = 'both'

if __name__ == '__main__':
    rospy.init_node('VisionArbitrator')
    try:
        ne = VisionArbitrator()
        rospy.spin()
    except rospy.ROSInterruptException: pass
