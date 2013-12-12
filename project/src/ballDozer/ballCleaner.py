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
        #Initialize vision arbitrator
        publisher = rospy.Publisher('/detectionSwitch', DetectionSwitcher)
        switcher = DetectionSwitcher()
        switcher.state = 2
        for i in xrange(0,25):
            switcher.header.stamp = rospy.Time.now()
            publisher.publish(switcher)

        self.debug = float(rospy.get_param('~debug', '0'))
        self.blueLandmark = int(rospy.get_param('~blueLandmark', '2'))
        self.greenLandmark = int(rospy.get_param('~greenLandmark', '3'))
        self.yellowLandmark = int(rospy.get_param('~yellowLandmark', '4'))
        self.purpleLandmark = int(rospy.get_param('~purpleLandmark', '5'))
        self.orangeLandmark = int(rospy.get_param('~orangeLandmark', '6'))

        rospy.Subscriber('ballCollectionMovement', Movement, self.setBallMovement)
        rospy.Subscriber('landmarkDetection', Movement, self.setLandmarkMovement)
        rospy.Subscriber('currentBallState', DetectionSwitcher, self.updateState)
        rospy.Subscriber('foundBallColor', DetectionSwitcher, self.updateCurrentBall)
        self.currentBall = None
        self.currentLandmark = None
        self.state = 0 #0 for ball collection, 1 for landmark finding/deploying

    def updateCurrentBall(self, detectionswitcher):
        if detectionswitcher.state == 1:
            self.currentBall = 'Orange'
        elif detectionswitcher.state == 2:
            self.currentBall = 'Purple'
        elif detectionswitcher.state == 3:
            self.currentBall = 'Yellow'
        elif detectionswitcher.state == 4:
            self.currentBall = 'Green'
        elif detectionswitcher.state == 5:
            self.currentBall = 'Blue'
        else:
            print('Error: ball color corresponding to #' + str(detectionswitcher.state) + ' not found')
        self.currentLandmark = self.mapColorToLandmarkNumber(self.currentBall)

    def mapColorToLandmarkNumber(self, color):
        if color == 'Orange':
            return self.orangeLandmark
        elif color == 'Purple':
            return self.purpleLandmark
        elif color == 'Green':
            return self.greenLandmark
        elif color == 'Yellow':
            return self.yellowLandmark
        elif color == 'Blue':
            return self.blueLandmark

    def updateState(self, currentState):
        publisher = rospy.Publisher('/detectionSwitch', DetectionSwitcher)
        switcher = DetectionSwitcher()
        print self.state
        if currentState.state == 0:
            switcher.state = 1
            self.state = 1
        elif currentState.state == 1:
            switcher.state = 2
            self.state = 0
        print self.state
        for i in xrange(0, 25):
            switcher.header.stamp  =rospy.Time.now()
            publisher.publish(switcher)

    def setBallMovement(self, move):
        if self.state == 0:
            publisher = rospy.Publisher('/ballCleanerOut', Movement)
            publisher.publish(move)

    def setLandmarkMovement(self, move):
        if self.state == 1:
            publisher = rospy.Publisher('/ballCleanerOut', Movement)
            publisher.publish(move)
            publisher2 = rospy.Publisher('/landmarkNumberOut', DetectionSwitcher)
            state = DetectionSwitcher()
            state.state = self.currentLandmark
            publisher2.publish(state)

    def debugPrint(self,string):
        if self.debug == 1:
            print string

if __name__ == '__main__':
    rospy.init_node('BallCleaner')
    try:
        ne = BallCleaner()
        rospy.spin()
    except rospy.ROSInterruptException: pass
