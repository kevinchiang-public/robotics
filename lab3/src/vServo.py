#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('landmarkSelfSim')
roslib.load_manifest('lab2')
from landmarkSelfSim.msg import landmarkLocation
from lab2.msg import Movement
import math

class CameraIntegrator():
    def __init__(self):
        self.debug = float(rospy.get_param('~debug', '1'))
        self.previousError=0
        self.P = float(rospy.get_param('~P','1'))
        self.D = float(rospy.get_param('~D','0'))
        #distance from camera to the front most point of hovercraft is 15 cm, the mimal camera   distance is 36 cm
        self.targetDistance = float(rospy.get_param('~Distance', '40'))
        self.move = Movement()
        rospy.Subscriber('/landmarkLocation', landmarkLocation, self.integrateRawValues) #Probably wrong

    def integrateRawValues(self, landmarkLoc):
        #Grab data from camera
        centerX=(landmarkLoc.xtop+landmarkLoc.xbottom)/2
        centerY=(landmarkLoc.ytop+landmarkLoc.ybottom)/2
        height=landmarkLoc.height
        code=landmarkLoc.code

        #Calculate actual distance using interpolated polynomial
        distance=self.calcDistance(height)

        #PI control for y axis only
        currentDistance=distance
        targetDistance=self.targetDistance
        r = self.P*(targetDistance-currentDistance)
        r = r + self.D*((targetDistance - currentDistance) - self.previousError)

        self.previousError = targetDistance - currentDistance

        #deadband
        if math.fabs(targetDistance - currentDistance) < 5:
            r = 0


        self.debugPrint("R: %6.2f Target Distance: %6.2f    Current Distance: %6.2f   Previous Error: %6.2f"%(-r, targetDistance,currentDistance,self.previousError))
        r = r/10.0

        if r > 1:
            r = 1
        elif r < -1:
            r = -1

        publisher = rospy.Publisher('/visualServoOut', Movement)
        self.move.theta = 0
        self.move.y = -r
        self.move.x = 0
        self.move.modType = 'Add'
        publisher.publish(self.move)

    def calcDistance(self, rawValue):
        returnVal = 7334.9 * math.pow(rawValue, -0.996)
        return returnVal

    def debugPrint(self, string):
        if self.debug == 1:
            print string


if __name__ == '__main__':
    rospy.init_node('CameraIntegrator')
    try:
        ne = CameraIntegrator()
        rospy.spin()
    except rospy.ROSInterruptException: pass
