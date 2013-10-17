#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('lab2')
from sensor_msgs.msg import Joy
from lab2.msg import MovementRaw, Movement
import math
from copy import deepcopy as deep
class AngleIntegrator():
    def __init__(self):
        self.debug = float(rospy.get_param('~debug', '0'))

        #Used to track button toggle states
        self.xButtonDepressed = False
        self.bButtonDepressed = False

        self.buttonTargetAngle=0
        self.rightBumperAngle=0
        self.leftBumperAngle=0
        #Used to determine whether to use input from joy or +- 90 degrees from x/b
        #Note: x/b adds to rotation starting at 0.  Doesn't add to current angle.
        self.useJoystick = False

        rospy.Subscriber('/joyOut', MovementRaw, self.interpretJoystick)

    def interpretJoystick(self,preMove):
        move = deep(preMove)
        publisher = rospy.Publisher('/angleIntegratorOut',Movement)
        xAxisL = move.xL
        yAxisL = move.yL
        xAxisR = move.xR
        yAxisR = move.yR
        xButton = move.xButton
        bButton = move.bButton
        leftBumperMag = move.bumperL
        rightBumperMag= move.bumperR

        #X/B button toggle logic
        if bButton == 1 and not self.bButtonDepressed:
            self.bButtonDepressed = True
            self.buttonTargetAngle += 90
        elif xButton == 1 and not self.xButtonDepressed:
            self.xButtonDepressed = True
            self.buttonTargetAngle += -90
        if bButton == 0 and self.bButtonDepressed:
            self.bButtonDepressed = False
        if xButton == 0 and self.xButtonDepressed:
            self.xButtonDepressed = False

        #Bumper logic (rotational spin using shoulders)
        #Right overrides left
        bumperMag = 0
        if rightBumperMag != 1:
            bumperMag = (1 - rightBumperMag)
        elif leftBumperMag != 1:
            bumperMag = (1 - leftBumperMag)

        #Get the arctangent of xAxis/yAxis to get the angle in radians.
        #Convert it to degrees and make it so that it goes from 0-360 starting
        #at the positive y axis (to match with the front of the hovercraft).
        #Uses extreme deadzone to makesure accidental rotations don't happen.
        magnitudeThreshold = 1
        magnitude = math.sqrt(xAxisL**2 + yAxisL**2)
        rotationalAngle = 0
        if magnitude >= magnitudeThreshold:
            rotationalAngle = round(math.atan2(xAxisL,yAxisL)*(180.0/3.141593),4)
            if (rotationalAngle > 0):
                rotationalAngle = rotationalAngle - 360
            rotationalAngle = math.fabs(rotationalAngle)

        #Ships off the message to the arbitrator
        #Joystick overrides button target commands
        moveOut = Movement()
        moveOut.modType = 'Bound'

        #Joystick Logic
        if magnitude >= magnitudeThreshold:
            #Reset button  upon hitting the joystick
            self.buttonTargetAngle = 0
            moveOut.theta = rotationalAngle
            moveOut.modType = 'Bound'

        #Trigger absolute rotation if trigger/bumper is held down
        if rightBumperMag != 1:
            self.rightBumperAngle = self.rightBumperAngle + (rightBumperMag * 100)
            self.buttonTargetAngle = 0
            moveOut.theta = self.rightBumperAngle
            moveOut.modType = 'Add'
            magnitude = 1
        else:
            self.rightBumperAngle = 0

        if leftBumperMag != 1:
            self.buttonTargetAngle = 0
            self.leftBumperAngle = self.leftBumperAngle + (leftBumperMag * 100)
            moveOut.theta = -1 * self.leftBumperAngle
            moveOut.modType = 'Add'
            magnitude = 1
        else:
            self.leftBumperAngle = 0

        #For 90 degree button rotations
        if math.fabs(self.buttonTargetAngle) > 0:
            magnitude = 1
            moveOut.theta = self.buttonTargetAngle
            moveOut.modType = 'Add'

        moveOut.x = xAxisR
        moveOut.y = yAxisR
        moveOut.mag = magnitude

        publisher.publish(moveOut)

        #Prints all information related to the integrator if need be
        if (self.debug == 1):
            print("xL: %6.2f  yL: %6.2f  Angle: %6.2f  Magnitude:%6.2f  "
                  "xR: %6.2f  yR: %6.2f Theta: %6.2f Button Target: %6.2f"
                  "rB: %6.2f  lB: %6.2f rM: %6.2f  lM: %6.2f " % (xAxisL,yAxisL,rotationalAngle,magnitude,xAxisR,yAxisR,moveOut.theta, self.buttonTargetAngle, self.rightBumperAngle, self.leftBumperAngle, rightBumperMag, leftBumperMag))

if __name__ == '__main__':
    rospy.init_node('AngleIntegrator')
    try:
        ne = AngleIntegrator()
        rospy.spin()
    except rospy.ROSInterruptException: pass
