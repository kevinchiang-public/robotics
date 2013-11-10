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
        self.ballPublisher = rospy.Publisher('/vis/Arbitrator/toBall',Image)
        self.landPublisher = rospy.Publisher('/vis/Arbitrator/toLand',Image)
        rospy.Subscriber('/vis/Arbitrator/cameraIn', Image, self.cameraCallback)
        rospy.Subscriber('/vis/Arbitrator/switcher', DetectionSwitcher, self.switcherCallback)

    def cameraCallback(self, image):
        if publishTarget is 'ballDetector':
            self.ballPublisher.publish(image)
        elif publishTarget is 'landDetector':
            self.landPublisher.publish(image)
    def switcherCallback(self, switcher):
        if switcher.state == 0:
            self.publishTarget = 'ballDetector'
        elif switcher.state == 1:
            self.publishTarget = 'landDetector'
        
if __name__ == '__main__':
    rospy.init_node('VisionArbitrator')
    try:
        ne = VisionArbitrator()
        rospy.spin()
    except rospy.ROSInterruptException: pass
