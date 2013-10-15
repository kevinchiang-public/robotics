#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('lab2')
from sensor_msgs.msg import Joy
from lab2.msg import MovementRaw, Movement
import math

class AngleIntegrator():
    def __init__(self):
        self.debug = float(rospy.get_param('~debug', '0'))

        #Used to track button toggle states
        self.xButtonDepressed = False
        self.bButtonDepressed = False

        #Used to determine whether to use input from joy or +- 90 degrees from x/b
        #Note: x/b adds to rotation starting at 0.  Doesn't add to current angle.
        self.useJoystick = False
        self.buttonTargetAngle = 0

        rospy.Subscriber('/joyOut', MovementRaw, self.interpretJoystick)

    def interpretJoystick(self,move):
        publisher = rospy.Publisher('/angleIntegratorOut',Movement)
        xAxisL = move.xL
        yAxisL = move.yL
        xAxisR = move.xR
        yAxisR = move.yR
        xButton = move.xButton
        bButton = move.bButton

        #X/B button toggle logic
        if bButton == 1 and not bButtonDepressed:
                bButtonDepressed = True
                self.buttonTargetAngle += 90
        elif xButton == 1 and not xDepressed:
                xButtonDepressed = True
                self.buttonTargetAngle += -90
        if bButton == 0 and bDepressed:
                bDepressed = False
        if xButton == 0 and xDepressed:
                xDepressed = False


        #Get the arctangent of xAxis/yAxis to get the angle in radians.
        #Convert it to degrees and make it so that it goes from 0-360 starting
        #at the positive y axis (to match with the front of the hovercraft).
        #Uses extreme deadzone to makesure accidental rotations don't happen.
        #We may want to incorporate the magnitude later
        #to control target velocity maybe (as a multiplier, perhaps?)
        magnitudeThreshold = 1
        magnitude = math.sqrt(xAxisL**2 + yAxisL**2)
        rotationalAngle = 0
        if magnitude >= magnitudeThreshold:
            rotationalAngle = round(math.atan2(xAxisL,yAxisL)*(180.0/3.141593),4)
            if (rotationalAngle > 0):
                rotationalAngle = rotationalAngle - 360
            rotationalAngle = math.fabs(rotationalAngle)

        #Prints all information related to the integrator if need be
        if (self.debug == 1):
            print("xL: %6.2f  yL: %6.2f  Angle: %6.2f  Magnitude:%6.2f  "
                  "xR: %6.2f  yR: %6.2f" % (xAxisL,yAxisL,rotationalAngle,
                                            magnitude,xAxisR,yAxisR))

        #Ships off the message to the arbitrator
        #Joystick overrides button target commands
        moveOut = Movement()
        if magnitude >= magnitudeThreshold:
            moveOut.theta = rotationalAngle
        else:
            moveOut.theta = self.buttonTargetAngle
        moveOut.x = xAxisR
        moveOut.y = yAxisR
        moveOut.mag = magnitude
        publisher.publish(moveOut)

if __name__ == '__main__':
    rospy.init_node('AngleIntegrator')
    try:
        ne = AngleIntegrator()
        rospy.spin()
    except rospy.ROSInterruptException: pass
