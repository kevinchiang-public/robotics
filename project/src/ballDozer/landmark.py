#!/usr/bin/env python
import rospy
import roslib
import math

from landmarkSelfSim.msg import landmarkVisible, landmarkLocation
from lab3.msg import Movement

class LandMark():
    def __init__(self):
        self.debug = float(rospy.get_param('~debug', '0'))
        self.currentLandmark = -2
        self.currentVisibleLMCode = -2
        self.isLandmarkInView = False
        self.landmarkLocation = None
        self.distanceToLandmark=1000
        self.landMarkX=None
        rospy.Subscriber('landmarkVisible',landmarkVisible, self.checkForLandmark)
        rospy.Subscriber('landmarkLocation', landmarkLocation, self.getDistanceToLandmark)
    
    #Updates the state w/r/t the landmark Visibility
    def checkForLandmark(self, landmarkVis):
        if landmarkVis.visible == 1:
            self.isLandmarkInView = True
        else:
            self.isLandmarkInView = False

    #Landmark Location is copied to local memory
    #Calculating the distance to landmark
    def getDistanceToLandMark(self, landmarkLoc):
        self.landmarkLocation = landmarkLoc
        self.currentVisibleLMCode = landmarkLoc.code
        self.landmarkX = (float(landmarkLoc.xtop)+float(landmarkLoc.xbottom))/2.0

        #Calculate actual distance using interpolated polynomial
        distance = self.distanceToLandmark
        if height != 0:
            distance = 7334.8 * math.pow(height, -0.996)
        self.distanceToLandmark = distance

        if self.distanceToLandmark < 75 and self.currentLandmark == landmarkLoc.code

    def moveTowardsLandmark(self):
        self.move(y=1, theta=(-(float(self.land

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
