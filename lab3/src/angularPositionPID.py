#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('lab3')
roslib.load_manifest('hovercraft')
from hovercraft.msg import Gyro
from sensor_msgs.msg import Joy
from lab3.msg import Movement
import math
from copy import deepcopy as deep
class AngularPositionPID():
    def __init__(self):
        self.debug = float(rospy.get_param('~debug', '1'))
        self.P = float(rospy.get_param('~P', '1'))
        self.D = float(rospy.get_param('~D', '0'))
        self.previousError = 0
        self.initialHeading = 0
        self.previousAngle=0
        #Initialize lift to false to make sure no thrusters fire on start
        self.movement = Movement()
        self.movement.lift = False
        self.first = True

        rospy.Subscriber('/arbitratorOut', Movement, self.arbitratorCallback)
        rospy.Subscriber('/hovercraft/Gyro', Gyro, self.gyroCallback)

    def debugPrint(self, string):
        if (self.debug == 1):
            print string

    def arbitratorCallback(self, move):
        self.movement = deep(move)

    def gyroCallback(self, gyro):
        currentAngle = deep(gyro.angle)

        #Initialize the target angle to the angle of the
        #hovercraft when it first turns on (prevents
        #spinning when the craft is launched)
        if self.first:
            self.movement.theta = currentAngle
            self.initialHeading = currentAngle
            self.previousAngle  = currentAngle
            self.first = False

        #Check to see if the magnitude is low.  If so,
        #set the target angle to the current angle (so if
        #the joy isn't depressed anymore, it stops moving).
        #Should be hard coded to 1 for any input that didn't
        #originate from the left joystick.
        if self.movement.mag < .5:
            self.movement.theta = self.previousAngle
        else:
            self.previousAngle = self.movement.theta

        targetAngle = self.movement.theta
        #Determine how the target angle should be affected given theta
        #(see Movement.msg for details)
        if self.movement.modType is 'Add':
            targetAngle = currentAngle + self.movement.theta
        elif self.movement.modType is 'Set':
            targetAngle = self.movement.theta
        elif self.movement.modType is 'Bound': #NOTE: CHECK THIS LOGIC
            targetAngle = self.movement.theta
        #Proportional and Derivative computations
        r = self.P*(targetAngle - currentAngle)
        r = r + self.D*((targetAngle - currentAngle)-self.previousError)
        self.previousError = targetAngle - currentAngle

        #Deadband
        if math.fabs(targetAngle - currentAngle) < 3:
            r = 0
        self.debugPrint("PosPID: Theta:{:6.2f} TargetAngle:{:6.2f}  GyroAngle:{:6.2f}  "
        "Diff: {:6.2f} ModType: {:10s} GyroRate: {:6.2f}".format(self.movement.theta, targetAngle,
                                                                 currentAngle, self.previousError,self.movement.modType,gyro.rate ))

        #Ship message off to VelocityPID
        move = Movement()
        pub = rospy.Publisher('/angularPositionOut',Movement)
        move.theta = r
        move.lift = self.movement.lift
        move.x = self.movement.x
        move.y = self.movement.y
        pub.publish(move)

if __name__ == '__main__':
        rospy.init_node('AngularPositionPID')
        try:
                ne = AngularPositionPID()
                rospy.spin()
        except rospy.ROSInterruptException: pass
