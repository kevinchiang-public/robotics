#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('lab3')
from landmarkSelfSim.msg import landmarkVisible, landmarkLocation
from ballDetector.msg import ballVisible, ballLocation
from geometry_msgs.msg import Vector3
from lab3.msg import Movement, Range, DetectionSwitcher

import math

class BallCleaner():
    def __init__(self):
        self.debug = float(rospy.get_param('~debug', '0'))
        self.blueLandmark = int(rospy.get_param('~blueLandmark', '2'))
        self.greenLandmark = int(rospy.get_param('~greenLandmark', '3'))
        self.yellowLandmark = int(rospy.get_param('~yellowLandmark', '4'))
        self.purpleLandmark = int(rospy.get_param('~purpleLandmark', '5'))
        self.orangeLandmark = int(rospy.get_param('~orangeLandmark', '6'))

        rospy.Subscriber('ballCollectionMovement', Movement, self.setBallMovement)
        rospy.Subscriber('landmarkDetection', Movement, self.setLandmarkMovement)
        rospy.Subscriber('currentBallState', DetectionSwitcher, self.updateState)
        self.state = 0 #0 for ball collection, 1 for landmark finding/deploying

        #Initialize vision arbitrator
        publisher = rospy.Publisher('/switcher/visArbitrator', DetectionSwitcher)
        switcher = DetectionSwitcher()
        switcher.state = 2
        for i in xrange(0,25):
            publisher.publish(switcher)

#TODO: Subscribe to the message published by ball.py to set the current landmark number
#given the current color
    def updateState(self, currentState):
        publisher = rospy.Publisher('/switcher/visArbitrator', DetectionSwitcher)
        switcher = DetectionSwitcher()
        if currentState.state == 0:
            switcher.state = 1
            self.state = 1
        elif currentState.state == 1:
            switcher.state = 2
            self.state = 0
        publisher.publish(switcher)

    def setBallMovement(self, move):
        if self.state == 0:
            publisher = rospy.Publisher('/ballCleanerOut', Movement)
            publisher.publish(move)

#TODO: Publish to landmark the target landmark we're trying to drive to
    def setLandmarkMovement(self, move):
        if self.state == 1:
            publisher = rospy.Publisher('/ballCleanerOut', Movement)
            publisher.publish(move)

    def debugPrint(self,string):
        if self.debug == 1:
            print string


if __name__ == '__main__':
    rospy.init_node('BallCleaner')
    try:
        ne = BallCleaner()
        rospy.spin()
    except rospy.ROSInterruptException: pass
