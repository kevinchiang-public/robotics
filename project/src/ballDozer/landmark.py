#!/usr/bin/env python
import rospy
import roslib
import math
roslib.load_manifest('landmarkSelfSim')
roslib.load_manifest('lab3')
roslib.load_manifest('hovercraft')
from hovercraft.msg import Thruster
from landmarkSelfSim.msg import landmarkVisible, landmarkLocation
from lab3.msg import Movement, DetectionSwitcher
from hoverboard.msg import PWMRaw
class LandMark():
    def __init__(self):
        self.debug = float(rospy.get_param('~debug', '0'))
        #Note: Increase above 40 with EXTREME caution
        self.firePower = float(rospy.get_param('~firePower', '40'))

        self.landmark1= -1
        self.currentLandmark = -2
        self.currentVisibleLMCode = -2
        self.isLandmarkInView = False
        self.landmarkLocation = None
        self.distanceToLandmark=1000
        self.targetReached = False
        self.landMarkX = None

        self.state=0

        rospy.Subscriber('landmarkVisible',landmarkVisible, self.checkForLandmark)
        rospy.Subscriber('landmarkLocation', landmarkLocation, self.getDistanceToLandmark)
        rospy.Subscriber('/landmarkNumberOut', DetectionSwitcher, self.updateTargetLandmark)

    def updateTargetLandmark(self, state):
        #The state corresponds to the landmark number
        self.landmark1 = state.state

    #NOTE: Please check the reset logic.
    def stateMachine(self):
        publisher = rospy.publisher('landmarkDetection',Movement)
        moveMessage = None
        if (self.state == 0): #Look for the landmark
            moveMessage = self.lookForLandmark(self.landmark1)
        elif (self.state == 1): #Move towards landmark
            moveMessage = self.moveTowardsLandmark
        else: #Reset to initial state
            moveMessage = self.move()
            self.currentLandmark = -2
            self.currentVisibleLMCode = -2
            self.isLandmarkInView = False
            self.landmarkLocation = None
            self.distanceToLandmark = 1000
            self.targetReached = False
            self.landMarkX = None
            self.state = 0

            donePublisher = rospy.publisher('currentBallState',  DetectionSwitcher) #Reuse of existing message type
            dState = DetectionSwitcher()
            dState.state = 1
            donePublisher.publish(dState)

        publisher.publish(moveMessage)

    #Updates the state w/r/t the landmark Visibility
    def checkForLandmark(self, landmarkVis):
        if landmarkVis.visible == 1:
            self.isLandmarkInView = True
        else:
            self.isLandmarkInView = False

    #Landmark Location is copied to local memory
    #Calculating the distance to landmark
    def getDistanceToLandmark(self, landmarkLoc):
        self.landmarkLocation = landmarkLoc
        self.currentVisibleLMCode = landmarkLoc.code
        self.landmarkX = (float(landmarkLoc.xtop)+float(landmarkLoc.xbottom))/2.0

        #Calculate actual distance using interpolated polynomial
        distance = self.distanceToLandmark
        if height != 0:
            distance = 7334.8 * math.pow(height, -0.996)
        self.distanceToLandmark = distance
        if self.distanceToLandmark < 75 and self.currentLandmark == landmarkLoc.code:
            self.targetReached = True
        else:
            self.targetReached = False

    #This is straight up copied from lab3 ballDozer
    def lookForLandmark(self, landmarkNum):
        if not self.isLandmarkInView or (self.currentVisibleLMCode != landmarkNum and self.currentVisibleLMCode != -1):
            self.debugPrint(str(self.isLandmarkInView) + " " + str(self.currentVisibleLMCode) + " " + str(landmarkNum))
            return self.move(theta=(-15))
        else:
            self.state += 1
            return self.move() #No arguments means stop

    #Move to landmark while centering it based on where the landmark location is found on the image
    def moveTowardsLandmark(self):
        if not self.targetReached:
            return self.move(y=1, theta=(-(float(self.landmarkX)-180.0)/10.0))
        else:
            state += 1
            return self.move()

    def launch(self):
        launchPub = rospy.Publisher('/hoverboard/PWMRaw',PWMRaw)
        FIRE = PWMRaw(channel=1, pwm=self.FirePower)
        for i in xrange(0,25):
            launchPub.publish(FIRE)
        self.state+=1
        #Stop firing the thruster
        OHGODSTOPFIRING = PWMRaw(channel=1, pwm=20)
        launchPub.publish(OHGODSTOPFIRING)
        return move()

    def move(self, x=0, y=0, theta=0):
        move = Movement()
        move.x = x
        move.y = y
        move.theta = theta
        move.modType = 'Add'
        return move

    def debugPrint(self, string):
        if self.debug == 1:
            print string

if __name__ == '__main__':
    rospy.init_node('LandMark')
    try:
        ne = LandMark()
        rospy.spin()
    except rospy.ROSInterruptException: pass
