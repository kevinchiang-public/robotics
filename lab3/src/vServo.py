#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('landmarkSelfSim')
roslib.load_manifest('lab3')
from landmarkSelfSim.msg import landmarkLocation
from lab3.msg import Movement
from lab3.msg import Distance
import math

class CameraIntegrator():
    def __init__(self):
        self.debug = float(rospy.get_param('~debug', '1'))
        self.previousYError=0
        self.previousXError=0
        self.PY = float(rospy.get_param('~Py','1'))
        self.DY = float(rospy.get_param('~Dy','0'))
        self.PX = float(rospy.get_param('~Px','1'))
        self.DX = float(rospy.get_param('~Dx','0'))
        self.height = 0
        self.code = -2

        self.YtargetDistance = float(rospy.get_param('~Distance', '80'))
        self.XtargetDistance = 0.0

        #The landmark to find.  If set to -1, it will follow any landmark visible
        #Two landmarks visible at once with -1 will result in undefined behavior
        self.targetLandmark = float(rospy.get_param('~LandmarkNumber', '-1'))
        self.move = Movement()
        rospy.Timer(rospy.Duration(.1), self.moveHovercraft)
        rospy.Subscriber('landmarkLocation', landmarkLocation, self.integrateRawValues)

        self.ticksSinceLandmarkSeen = 0
    #Set on a timer to continue moving the hovercraft
    #in the last set direction in case the landmark is lost
    def moveHovercraft(self, event):
        publisher = rospy.Publisher('/visualServoOut', Movement)
        if (self.ticksSinceLandmarkSeen < 25):
            publisher.publish(self.move)
        else:
            noMove = Movement()
            noMove.x = 0
            noMove.y = 0
            noMove.theta = 0
            noMove.modType = 'Add'
            publisher.publish(noMove)
        self.ticksSinceLandmarkSeen += 1

    def integrateRawValues(self, landmarkLoc):
        self.ticksSinceLandmarkSeen = 0
        #Grab data from camera
        height=landmarkLoc.height
        code=landmarkLoc.code

        #Calculate actual distance using interpolated polynomial
        distance = 0

        if height != 0:
            distance=self.calcDistance(height)
            #publish distance to plot
            if (distance < 300):
                publisher1 = rospy.Publisher('/visualServodistance', Distance)
                ydistance=Distance()
                ydistance.header.stamp=rospy.Time.now()
                ydistance.Distance=distance
                publisher1.publish(ydistance)

        #Get the target translational thrust in Y direction
        yR = self.yPID(distance)
        #If we get a bad height reading, just publish the previous move message
        if (height != 0 and (code == self.targetLandmark or self.targetLandmark == -1)):
            #centerDistance = landmarkLoc.centerDistance #Attempt to get center distance of entire landmark
            centerDistance = 160 - (landmarkLoc.xbottom)
            self.move.theta = 0#-(self.getTargetAngle(centerDistance)) #If rotation is backwards, make negative
            self.move.y = -yR
            self.move.x = 0
            self.move.modType = 'Add'

    def yPID(self, currentDistance):
        #PD control for y axis only
        targetDistance=self.YtargetDistance
        r = self.PY*(targetDistance-currentDistance)
        r = r + self.DY*((targetDistance - currentDistance) - self.previousYError)
        self.previousYError = targetDistance - currentDistance

        #Deadband
        if math.fabs(targetDistance - currentDistance) < 5:
            r = 0

        self.debugPrint("yR: %6.2f Target Distance: %6.2f    Current Distance: %6.2f "
                        "Previous Error: %6.2f "%
                        (-r, targetDistance,currentDistance,self.previousYError))

        #Arbitrary division to get r into appropriate thruster range
        #(determined via some trial-and-error experimentation)
        r = r/50.0
        if r > 1:
            r = 1
        elif r < -1:
            r = -1
        return r

    #Determines the target angle of the hovercraft based on
    #the difference in distance between the center of the
    #landmark and the hardcoded 320 width camera image
    def getTargetAngle(self, centerDifference):
        #PD control for theta
        targetDistance=self.XtargetDistance
        r = self.PX*(targetDistance-centerDifference)
        r = r + self.DX*((targetDistance - centerDifference) - self.previousXError)
        self.previousXError = targetDistance - centerDifference

        #Deadband
        if math.fabs(targetDistance - centerDifference) < 5:
            r = 0

        #Arbitrary division to get r into appropriate thruster range
        #(determined via some trial-and-error experimentation)
        #r = r/20.0
        self.debugPrint(str(r))
        return r

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
