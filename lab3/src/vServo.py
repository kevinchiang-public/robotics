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
        self.previousYError=0
        self.P = float(rospy.get_param('~P','1'))
        self.D = float(rospy.get_param('~D','0'))
        self.centerX = 0
        self.centerY = 0
        self.height = 0
        self.code = -2

        #distance from camera to the front most point of hovercraft is 15 cm, the mimal camera   distance is 36 cm
        self.targetDistance = float(rospy.get_param('~Distance', '40'))

        #The landmark to find.  If set to -1, it will follow any landmark visible
        #Two landmarks visible at once with -1 will result in undefined behavior
        self.targetLandmark = float(rospy.get_param('~LandmarkNumber', '-1'))
        self.move = Movement()
        rospy.Timer(rospy.Duration(.1), self.moveHovercraft)
        rospy.Subscriber('/landmarkLocation', landmarkLocation, self.integrateRawValues)

    #Set on a timer to continue moving the hovercraft
    #in the last set direction in case the landmark is lost
    def moveHovercraft(self):
        publisher.publish(self.move)

    def integrateRawValues(self, landmarkLoc):
        #Grab data from camera
        centerX=(landmarkLoc.xtop+landmarkLoc.xbottom)/2.0
        centerY=(landmarkLoc.ytop+landmarkLoc.ybottom)/2.0
        height=landmarkLoc.height
        code=landmarkLoc.code

        #Calculate actual distance using interpolated polynomial
        distance = 0
        if height != 0:
            distance=self.calcDistance(height)

        #Get the target translational thrust in Y direction
        yR = yPID(distance)

        #If we get a bad height reading, just publish the previous move message
        publisher = rospy.Publisher('/visualServoOut', Movement)
        if (height != 0 and (code == targetLandmark or targetLandmark == -1):
            self.move.theta = self.getTargetAngle(0) #Change this later (when we figure out how to do it)
            self.move.y = -r
            self.move.x = 0
            self.move.modType = 'Add'

    def yPID(self, currentDistance):
        #PD control for y axis only
        targetDistance=self.targetDistance
        r = self.P*(targetDistance-currentDistance)
        r = r + self.D*((targetDistance - currentDistance) - self.previousYError)
        self.previousYError = targetDistance - currentDistance

        #Deadband
        if math.fabs(targetDistance - currentDistance) < 5:
            r = 0

        self.debugPrint("yR: %6.2f Target Distance: %6.2f    Current Distance: %6.2f "
                        "Previous Error: %6.2f Height: %6.2f"%
                        (-r, targetDistance,currentDistance,self.previousError, height))

        #Arbitrary division to get r into appropriate thruster range
        #(determined via some trial-and-error experimentation)
        r = r/50.0
        if r > 1:
            r = 1
        elif r < -1:
            r = -1
        return r

    #Determines the target angle of the hovercraft based on
    #the center of the landmark w.r.t. the entire camera image
    def getTargetAngle(self, center): #TODO: IMPLEMENT
            return 0.0

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
