#!/usr/bin/env python
import rospy
import roslib
import math

from lab3.msg import Movement

class LandMark():
    def __init__(self):
        self.debug = float(rospy.get_param('~debug', '0'))

        rospy.Subscriber('landmarkVisible',landmarkVisible, self.checkForLandmark)

    def checkForLandmark(self, landmarkVis):
        pass

    def moveTowardsLandmark(self)

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
