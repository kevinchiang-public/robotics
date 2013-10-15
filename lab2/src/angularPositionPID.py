#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('lab2')
roslib.load_manifest('hovercraft')
from hovercraft.msg import Gyro
from sensor_msgs.msg import Joy
from lab2.msg import Movement
import math

class AngularPositionPID():
    def __init__(self):
        self.debug = float(rospy.get_param('~debug', '0'))
        self.P = float(rospy.get_param('~P', '.01'))
        self.D = float(rospy.get_param('~D', '.01'))

        self.previousError = 0

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
        self.movement = move

    def gyroCallback(self, gyro):
        #Initialize the target angle to the angle of the
        #hovercraft when it first turns on (prevents
        #spinning when the craft is launched)
        if self.first:
            self.movement.theta = gyro.angle
            self.first = False

        #Check to see if the magnitude is low.  If so,
        #set the target angle to the current angle (so if
        #the joy isn't depressed anymore, it stops moving)
        if self.movement.magnitude < .5:
            self.movement.theta = gyro.angle

        targetAngle = self.movement.theta
        #Proportional and Derivative computations
        r = self.P*(targetAngle - gyro.angle)
        r = r + self.D*((targetAngle - gyro.angle)-self.previousError)
        self.previousError = targetAngle - gyro.angle

        #Deadband
        if math.fabs(targetAngle - gyro.angle) < 3:
            r = 0
        self.debugPrint("PosPID: TargetAngle:{:6.2f}  GyroAngle:{:6.2f}  "
                        "Diff: {:6.2f}".format(targetAngle,
                                                gyro.angle, self.previousError))

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
