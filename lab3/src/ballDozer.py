#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('lab3')
from landmarkSelfSim.msg import landmarkVisible, landmarkLocation
from ballDetector.msg import ballVisible, ballLocation
from geometry_msgs.msg import Vector3
from lab3.msg import Movement, Range

import math

class BallDozer():
    def __init__(self):
        self.debug = float(rospy.get_param('~debug', '0'))
        self.color1 = rospy.get_param('~color1', 'red')
        self.color2 = rospy.get_param('~color2', 'green')
        self.ballColor1Set = False
        self.ballColor2Set = False
        self.landmarkForColor1 = float(rospy.get_param('~landmark1', '1'))
        self.landmarkForColor2 = float(rospy.get_param('~landmark2', '1'))

        #Used when we don't have any action to publish
        self.haltMove = Movement()
        self.haltMove.theta = 0
        self.haltMove.x = 0
        self.haltMove.y = 0
        self.haltMove.modType = 'Add'

        self.state = 0
        self.prevState = 0 #Used to print debug messages when state changes
        self.isBallCaptured = False #Uses the IR range finders to see if the ball is in the prong
        self.isBallInView = False #Uses the camera to see if a ball is visible in the scene

        self.isLandmarkInView = False #Uses camera to see if landmark is visible in the scene
        self.distanceToLandmark = 0

        rospy.Timer(rospy.Duration(.1), self.stateBasedMove) #Updates movement based on current state
        rospy.Subscriber('/rangeInfo', Range, self.checkForBallCaptured) #Checks for ball in prong via IR sensors
        rospy.Subscriber('ballVisible', ballVisible, self.checkForBallVisibility)
        rospy.Subscriber('landmarkLocation', landmarkLocation, self.getDistanceToLandmark)
        rospy.Subscriber('landmarkVisible', landmarkVisible, self.checkForLandmark)  #Checks for landmark

    def checkForBallVisibility(self, ballVis):
        if (ballVis.visible == 1):
            self.isBallInView = True
        else:
            self.isBallInView = False

    def getDistanceToLandmark(self, landmarkInfo):
        #Grab data from camera
        height=landmarkLoc.height
        code=landmarkLoc.code

        #Calculate actual distance using interpolated polynomial
        distance = self.distanceToLandmark
        if height != 0:
            distance = 7334.9 * math.pow(height, -0.996)
        self.distanceToLandmark = distance

    def searchForBall(self):
        if (self.isBallCaptured):
            self.state += 2
            return self.haltMove
        if (not self.isBallInView):
            return self.spin()
        else:
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
            move.y = 1
            move.x = 0
            move.theta = 0
            move.modType = 'Add'
            return move

    def lookForLandmark(self, landmarkNum):
        if not self.isBallCaptured:  #Lost the ball from the prong.  Find it.  Reacquire.
            self.state -= 1
            return self.haltMove
        elif not self.isLandmarkInView:
            return self.spin()
        else:
            self.state += 1
            return self.haltMove

    def moveTowardsLandmark(self, landmarkNum):
        #Incorporate landmark Number we're looking for
        if not self.isBallCaptured:
            self.state -= 1
            return self.haltMove
        elif not self.isLandmarkInView:
            state -= 1
            return self.haltMove
        else:
            move = Movement()
            move.y = 1
            move.x = 0
            move.theta = 0
            move.modType = 'Add'
            return move

    def checkForBallCaptured(self, rangeInfo):
        lDistCM = rangeInfo.leftDistanceCM
        rDistCM = rangeInfo.rightDistanceCM
        self.debugPrint("lDistCM: "+str(lDistCM)+ " rDistCM: "+str(rDistCM))
        if (lDistCM < 12 and lDistCM > 0) or (rDistCM < 12 and rDistCM > 0):
            self.debugPrint("Captured")
            self.isBallCaptured = True
        else:
            self.debugPrint("Not Captured")
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
        move.theta = -10
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
            hsvColor.x = 0 #H value
            hsvColor.y = 154 #S value
            hsvColor.z = 101 #V value
        elif colorString == 'yellow':
            hsvColor.x = 78 #H value
            hsvColor.y = 112 #S value
            hsvColor.z = 97 #V value
        else:
            self.debugPrint("Unknown Color: " + str(colorString))
        return hsvColor

    #TODO: Find correct parameters to recognize HSV values for each of the following
    def getHSVHighThreshold(self, colorString):
        hsvColor = Vector3()
        if colorString == 'red':
            hsvColor.x = 122 #H value
            hsvColor.y = 186 #S value
            hsvColor.z = 255 #V value
        elif colorString == 'blue':
            hsvColor.x = 17 #H value
            hsvColor.y = 255 #S value
            hsvColor.z = 249 #V value
        elif colorString == 'yellow':
            hsvColor.x = 97 #H value
            hsvColor.y = 163 #S value
            hsvColor.z = 223 #V value
        else:
            self.debugPrint("Unknown Color: " + str(colorString))
        return hsvColor

    def stateBasedMove(self, timerInfo):
        movePublish = rospy.Publisher('/ballDozerOut', Movement)
        move = Movement()

        #Print the state change
        if self.state != self.prevState:
            self.debugPrint(self.stateNumToWord())
            self.prevState = self.state
        if self.state == 0:
            self.setBallColor(1)
            move = self.searchForBall()
        elif self.state == 1:
            move = self.collectBall()
        elif self.state == 2:
            move = self.lookForLandmark(1)
        elif self.state == 3:
            move = self.moveTowardsLandmark(1)
        elif self.state == 4:
            self.setBallColor(2)
            move = self.searchForBall()
        elif self.state == 5:
            move = self.collectBall()
        elif self.state == 6:
            move = self.lookForLandmark(2)
        elif self.state == 7:
            move = self.moveTowardsLandmark(2)
        else:
            debugPrint("Unknown state!")
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
            return "Ball delivered.  Looking for ball 2"
        elif self.state == 5:
            return "Moving towards found ball 2"
        elif self.state == 6:
            return "Ball 2 collected.  Looking for landmark 2"
        elif self.state == 7:
            return "Ball 2 delivered.  Donezo."

    def setBallColor(self, ballNumber):
        lowPublisherUpdate = rospy.Publisher('/thresh/low', Vector3)
        highPublisherUpdate = rospy.Publisher('/thresh/high', Vector3)
        if ballNumber == 1 and not self.ballColor1Set:
            lowPublisherUpdate.publish(self.getHSVLowThreshold(self.color1))
            highPublisherUpdate.publish(self.getHSVHighThreshold(self.color1))
            self.ballColor1Set = True
        if ballNumber == 2 and not self.ballColor2Set:
            lowPublisherUpdate.publish(self.getHSVLowThreshold(self.color2))
            highPublisherUpdate.publish(self.getHSVHighThreshold(self.color2))
            self.ballColor2Set = True
    def debugPrint(self,string):
        if self.debug == 1:
            print string


if __name__ == '__main__':
    rospy.init_node('BallDozer')
    try:
        ne = BallDozer()
        rospy.spin()
    except rospy.ROSInterruptException: pass
