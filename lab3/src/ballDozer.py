#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('lab3')

import math, geometry_msgs.Vector3

class BallDozer():
    def __init__(self):
        self.debug = float(rospy.get_param('~debug', '0'))
        self.color1 = rospy.get_param('~color1', 'red')
        self.color2 = rospy.get_param('~color2', 'green')
        self.landmarkForColor1 = float(rospy.get_param('~landmark1', '1'))
        self.landmarkForColor2 = float(rospy.get_param('~landmark2', '1'))

        self.move = Movement()
        self.state = 0
        self.prevState = 0 #Used to print debug messages when state changes
        rospy.Timer(rospy.Duration(.1), self.stateBasedMove)
        #Need a callback for IR range finders to check if bal is caputred for collectBall, lookForLandmark, and moveTowardsLandmark functions

        #Need another callback to check if ball was found, and if landmark was found (simple functions, not state based)

    def searchForBall(self):

    def collectBall(self):

    def lookForLandmark(self, landmarkNum):

    def moveTowardsLandmark(self, landmarkNum):

    def spin(self):
        #Update movement to spin
        self.move.modType = 'Add'
        self.move.theta = 3
        self.move.x = 0
        self.move.y = 0

    #TODO: Find correct parameters to recognize HSV values for each of the following
    def getHSVLowThreshold(self, colorString):
        hsvColor = Vector3()
        if colorString is 'red':
            hsvColor.x = 0 #H value
            hsvColor.y = 0 #S value
            hsvColor.z = 0 #V value
        elif colorString is 'green':
            hsvColor.x = 0 #H value
            hsvColor.y = 0 #S value
            hsvColor.z = 0 #V value
        elif colorString is 'blue':
            hsvColor.x = 0 #H value
            hsvColor.y = 0 #S value
            hsvColor.z = 0 #V value
        elif colorString is 'orange':
            hsvColor.x = 0 #H value
            hsvColor.y = 0 #S value
            hsvColor.z = 0 #V value
        elif colorString is 'yellow':
            hsvColor.x = 0 #H value
            hsvColor.y = 0 #S value
            hsvColor.z = 0 #V value
        elif colorString is 'purple':
            hsvColor.x = 0 #H value
            hsvColor.y = 0 #S value
            hsvColor.z = 0 #V value
        return hsvColor

    #TODO: Find correct parameters to recognize HSV values for each of the following
    def getHSVHighThreshold(self, colorString):
        hsvColor = Vector3()
        if colorString is 'red':
            hsvColor.x = 0 #H value
            hsvColor.y = 0 #S value
            hsvColor.z = 0 #V value
        elif colorString is 'green':
            hsvColor.x = 0 #H value
            hsvColor.y = 0 #S value
            hsvColor.z = 0 #V value
        elif colorString is 'blue':
            hsvColor.x = 0 #H value
            hsvColor.y = 0 #S value
            hsvColor.z = 0 #V value
        elif colorString is 'orange':
            hsvColor.x = 0 #H value
            hsvColor.y = 0 #S value
            hsvColor.z = 0 #V value
        elif colorString is 'yellow':
            hsvColor.x = 0 #H value
            hsvColor.y = 0 #S value
            hsvColor.z = 0 #V value
        elif colorString is 'purple':
            hsvColor.x = 0 #H value
            hsvColor.y = 0 #S value
            hsvColor.z = 0 #V value
        return hsvColor

    def stateBasedMove(self):
        movePublish = rospy.Publisher('/ballDozerOut', Movement)
        move = Movement()
        #Print the state change
        if self.state != self.prevState:
            debugPrint(stateNumToWord)
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
        if ballNumber == 1:
            lowPublisherUpdate.publish(self.getHSVLowThreshold(self.color1))
            highPublisherUpdate.publish(self.getHSVHighThreshold(self.color1))
        if ballNumber == 2:
            lowPublisherUpdate.publish(self.getHSVLowThreshold(self.color2))
            highPublisherUpdate.publish(self.getHSVHighThreshold(self.color2))

    def debugPrint(self,string):
        if self.debug == 1:
            print string


if __name__ == '__main__':
    rospy.init_node('BallDozer')
    try:
        ne = BallDozer()
        rospy.spin()
    except rospy.ROSInterruptException: pass
