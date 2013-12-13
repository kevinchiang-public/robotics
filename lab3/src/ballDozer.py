#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('lab3')
from landmarkSelfSim.msg import landmarkVisible, landmarkLocation
from ballDetector.msg import ballVisible, ballLocation
from geometry_msgs.msg import Vector3
from lab3.msg import Movement, Range, DetectionSwitcher

import math

class BallDozer():
    def __init__(self):
        self.debug = float(rospy.get_param('~debug', '0'))
        self.color1 = rospy.get_param('~color1', 'blue')
        self.color2 = rospy.get_param('~color2', 'red')

        #Used as a boolean and loop iterator.  We need to publish to thresh/high and low
        #multiple times for the new color to actually get set
        self.ballColor1Set = 0
        self.ballColor2Set = 0

        self.landmarkForColor1 = float(rospy.get_param('~landmark1', '1'))
        self.landmarkForColor2 = float(rospy.get_param('~landmark2', '2'))
        self.currentLandmark = self.landmarkForColor1

        self.detectionPublisher = rospy.Publisher('/detectionSwitch',DetectionSwitcher)
        self.detectionSwitch = DetectionSwitcher()
        self.landmarkX = 0

        #Used when we're transitioning between states
        self.haltMove = Movement()
        self.haltMove.theta = 0
        self.haltMove.x = 0
        self.haltMove.y = 0
        self.haltMove.modType = 'Add'

        self.ballLocation = ballLocation()
        self.targetReached = False
        self.state = 0
        self.prevState = 0 #Used to print debug messages when state changes
        self.isBallCaptured = False #Uses the IR range finders to see if the ball is in the prong
        self.isBallInView = False #Uses the camera to see if a ball is visible in the scene
        self.isLandmarkInView = False #Uses camera to see if landmark is visible in the scene
        self.distanceToLandmark = 300
        self.currentVisibleLMCode = -2
        self.switchedToSecond = 0

        rospy.Timer(rospy.Duration(.1), self.stateBasedMove) #Updates movement based on current state
        rospy.Subscriber('/rangeInfo', Range, self.checkForBallCaptured) #Checks for ball in prong via IR sensors
        rospy.Subscriber('ballVisible', ballVisible, self.checkForBallVisibility)
        rospy.Subscriber('/ballLocation',ballLocation,self.getBallLocation)
        rospy.Subscriber('landmarkLocation', landmarkLocation, self.getDistanceToLandmark)
        rospy.Subscriber('landmarkVisible', landmarkVisible, self.checkForLandmark)  #Checks for landmark

    def getBallLocation(self,ballLocation):
        self.ballLocation = ballLocation

    def checkForBallVisibility(self, ballVis):
        if (ballVis.visible == 1):
            self.isBallInView = True
        else:
            self.isBallInView = False

    def getDistanceToLandmark(self, landmarkLoc):
        #Grab data from camera
        height=landmarkLoc.height
        self.currentVisibleLMCode=landmarkLoc.code
        self.landmarkX = (float(landmarkLoc.xtop)+float(landmarkLoc.xbottom))/2.0

        #Calculate actual distance using interpolated polynomial
        distance = self.distanceToLandmark
        if height != 0:
            distance = 7334.9 * math.pow(height, -0.996)
        self.distanceToLandmark = distance
        if self.distanceToLandmark < 75 and self.currentLandmark == landmarkLoc.code:
            self.targetReached = True
        else:
            self.targetReached = False

    def searchForBall(self):
        offset = 0  #Unused, but remains down about 7 lines
        if (self.isBallCaptured):
            self.state += 2
            return self.haltMove
        if (not self.isBallInView):
            return self.spin()
        elif self.isBallInView and not (self.ballLocation.x < (20+offset) and self.ballLocation.x > (-20+offset)):
            return self.spin()
        #We want it only to collect the ball when it is in the center of the image
        elif self.isBallInView and self.ballLocation.x < (20+offset) and self.ballLocation.x > (-20+offset):
            self.state += 1 #Found the ball
            return self.haltMove

    def collectBall(self):
        if not self.isBallInView:
            self.state -= 1 #Might run into trouble here if it gets decremented twice in a row
            return self.haltMove
        elif self.isBallCaptured:
            self.state += 1 #Ball is in the doohickey
            return self.haltMove
        else:
            move = Movement()
            move.y = .4
            move.x = 0
            #if self.ballLocation.x > 20:
            move.theta = -self.ballLocation.x/2
            move.modType = 'Add'
            return move

    def lookForLandmark(self, landmarkNum):
        '''
        if not self.isBallCaptured:  #Lost the ball from the prong.  Find it.  Reacquire.
            self.state -= 1
            return self.haltMove
        '''
        #Make sure the visible landmark is the one we need, or out of range to identify
        #If it;s out of range, move closer to investigate (i.e., assume it's correct and
        #backtrack a state if it's not)
        #self.debugPrint(str(self.isLandmarkInView) + " " + str(self.currentVisibleLMCode) + " " + str(landmarkNum))
        if not self.isLandmarkInView or (self.currentVisibleLMCode != landmarkNum and self.currentVisibleLMCode != -1):
            return self.spin()
        else:
            self.state += 1
            return self.haltMove

    def moveTowardsLandmark(self, landmarkNum):
        #Incorporate landmark Number we're looking for
        '''
        if not self.isBallCaptured:
            self.state -= 1
            return self.haltMove
        '''
        if not self.isLandmarkInView or (self.currentVisibleLMCode != landmarkNum and self.currentVisibleLMCode != -1):
            self.state -= 1
            return self.haltMove
        elif self.isLandmarkInView and not self.targetReached:
            move = Movement()
            move.y = 1
            move.x = 0
            move.theta = -(float(self.landmarkX)-180.)/10.
            move.modType = 'Add'
            return move
        elif self.isLandmarkInView and self.targetReached:
            self.state = 9 #Switch to ball kick out state
            return self.haltMove

    def checkForBallCaptured(self, rangeInfo):
        lDistCM = rangeInfo.leftDistanceCM
        rDistCM = rangeInfo.rightDistanceCM
        if (lDistCM < 9 and lDistCM > 0) or (rDistCM < 15 and rDistCM > 0):
            self.isBallCaptured = True
        else:
            self.isBallCaptured = False

    def checkForLandmark(self,landmarkVis):
        #Updates the state w.r.t the landmark visibility
        if landmarkVis.visible == 1:
            self.isLandmarkInView = True
        else:
            self.isLandmarkInView = False

    def spin(self):
        #Update movement to spin
        move = Movement()
        move.modType = 'Add'
        move.theta = -15
        move.x = 0
        move.y = 0
        return move

    def getHSVLowThreshold(self, colorString):
        hsvColor = Vector3()
        if colorString == 'red':
            hsvColor.x = 109 #H value
            hsvColor.y = 81 #S value
            hsvColor.z = 115 #V value
        elif colorString == 'blue':
            hsvColor.x = 1 #H value
            hsvColor.y = 155 #S value
            hsvColor.z = 102 #V value
        elif colorString == 'yellow':
            hsvColor.x = 78 #H value
            hsvColor.y = 112 #S value
            hsvColor.z = 97 #V value
        else:
            self.debugPrint("Unknown Color: " + str(colorString))
        return hsvColor

    def getHSVHighThreshold(self, colorString):
        hsvColor = Vector3()
        if colorString == 'red':
            hsvColor.x = 122 #H value
            hsvColor.y = 186 #S value
            hsvColor.z = 255 #V value
        elif colorString == 'blue':
            hsvColor.x = 18 #H value
            hsvColor.y = 256 #S value
            hsvColor.z = 250 #V value
        elif colorString == 'yellow':
            hsvColor.x = 97 #H value
            hsvColor.y = 163 #S value
            hsvColor.z = 223 #V value
        else:
            self.debugPrint("Unknown Color: " + str(colorString))
        return hsvColor

    def kickBallOut(self):
        #A sleep so we can pick the ball up.  The backwards momentum wasn't fast enough to kick the ball out
        rospy.sleep(5)
        #Keep moving backwards until the ball isn't in the hook anymore
        if self.isBallCaptured:
            move = Movement()
            move.y = -1
            move.x = 0
            move.theta = 0
            move.modType = 'Add'
            return move
        else:
            self.state = 4
            return self.haltMove

    def stateBasedMove(self, timerInfo):
        movePublish = rospy.Publisher('/ballDozerOut', Movement)
        move = Movement()

        self.detectionSwitch.header.stamp = rospy.Time.now()
        #Print the state change
        if self.state != self.prevState:
            self.debugPrint(self.stateNumToWord())
            self.prevState = self.state
        if self.state == 0:
            self.detectionSwitch.state = 2
            self.setBallColor(1)
            move = self.searchForBall()
        elif self.state == 1:
            move = self.collectBall()
        elif self.state == 2:
            #Switch to landmark Detection Mode
            self.detectionSwitch.state = 1
            move = self.lookForLandmark(self.landmarkForColor1)
        elif self.state == 3:
            move = self.moveTowardsLandmark(self.landmarkForColor1)
        elif self.state == 9:
            move = self.kickBallOut()
            self.detectionSwitch.state = 2
            #reset variables
            if self.switchedToSecond < 20:
                    self.isBallCaptured = False #Uses the IR range finders to see if the ball is in the prong
                    self.isBallInView = False #Uses the camera to see if a ball is visible in the scene
                    self.isLandmarkInView = False #Uses camera to see if landmark is visible in the scene
                    self.targetReached = False
                    self.distanceToLandmark = 300
                    self.currentVisibleLMCode = -2
                    self.landmarkForColor1 = self.landmarkForColor2
        elif self.state == 4:
            #reset variables
            if not self.switchedToSecond < 20:
                    self.isBallCaptured = False #Uses the IR range finders to see if the ball is in the prong
                    self.isBallInView = False #Uses the camera to see if a ball is visible in the scene
                    self.isLandmarkInView = False #Uses camera to see if landmark is visible in the scene
                    self.targetReached = False
                    self.switchedToSecond += 1
                    self.distanceToLandmark = 300
                    self.currentVisibleLMCode = -2

            #Onward, ho!  To the second stage of our state machine
            self.setBallColor(2)
            self.currentLandmark = self.landmarkForColor2

            #Switch to ball Detection Mode
            self.detectionSwitch.state = 2
            move = self.searchForBall()
        elif self.state == 5:
            move = self.collectBall()
        elif self.state == 6:
            #Switch to landmark Detection Mode
            self.detectionSwitch.state = 1
            move = self.lookForLandmark(self.landmarkForColor2)
        elif self.state == 7:
            move = self.moveTowardsLandmark(self.landmarkForColor2)
            self.state += 1
        else:
            self.state = 0
            self.debugPrint("Resetting state to 0!")
        self.detectionPublisher.publish(self.detectionSwitch)
        movePublish.publish(move)

    def stateNumToWord(self):
        if self.state == 0:
            return "Searching For Ball 1"
        elif self.state == 1:
            return "Moving towards found ball 1"
        elif self.state == 2:
            return "Ball 1 Collected.  Looking for landmark 1"
        elif self.state == 3:
            return "Moving towards found landmark 1"
        elif self.state == 4:
            return "Ball 1 delivered.  Looking for ball 2"
        elif self.state == 5:
            return "Moving towards found ball 2"
        elif self.state == 6:
            return "Ball 2 collected.  Looking for landmark 2"
        elif self.state == 7:
            return "Moving towards found landmark 2."
        elif self.state == 8:
            return "Ball 2 delivered.  Donezo"
        elif self.state == 9:
            return "Kicking ball 1 out of the prongs."

    def setBallColor(self, ballNumber):
        lowPublisherUpdate = rospy.Publisher('thresh/low', Vector3)
        highPublisherUpdate = rospy.Publisher('thresh/high', Vector3)
        #I have no idea why, but publishing once doesn't work
        loopIters = 25
        if ballNumber == 1 and self.ballColor1Set < loopIters:
            vec3 = self.getHSVLowThreshold(self.color1)
            lowPublisherUpdate.publish(x=vec3.x, y=vec3.y, z=vec3.z)
            vec3 = self.getHSVHighThreshold(self.color1)
            highPublisherUpdate.publish(x=vec3.x,y=vec3.y,z=vec3.z)
            self.ballColor1Set += 1
        elif ballNumber == 1 and self.ballColor1Set == loopIters:
            self.ballColor1Set += 1
            self.debugPrint("threshold set to " + self.color1)

        if ballNumber == 2 and self.ballColor2Set < loopIters:
            vec3 = self.getHSVLowThreshold(self.color2)
            lowPublisherUpdate.publish(x=vec3.x, y=vec3.y, z=vec3.z)
            vec3 = self.getHSVHighThreshold(self.color2)
            highPublisherUpdate.publish(x=vec3.x,y=vec3.y,z=vec3.z)
            self.ballColor2Set += 1
        elif ballNumber == 2 and self.ballColor2Set == loopIters:
            self.ballColor2Set += 1
            self.debugPrint("threshold set to " + self.color2)

    def debugPrint(self,string):
        if self.debug == 1:
            print string


if __name__ == '__main__':
    rospy.init_node('BallDozer')
    try:
        ne = BallDozer()
        rospy.spin()
    except rospy.ROSInterruptException: pass
