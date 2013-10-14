#!/usr/bin/env python
import rospy
import roslib
import math
roslib.load_manifest('lab2')
roslib.load_manifest('hovercraft')
from hovercraft.msg import Gyro
from lab2.msg import Movement

class AngularVelocityPID():
    def __init__(self):
        self.debug = float(rospy.get_param('~debug', '0'))
        self.targetRate = 0
        self.previousError = 0
        self.movePass = Movement()
        rospy.init_node('AngularVelocityPID')
        rospy.Subscriber('/hovercraft/Gyro', Gyro, self.gyroCallback)
        rospy.Subscriber('/angularPositionOut', Movement, self.positionPIDCallback)

    def printDebug(self, stringToPrint):
        if self.debug == 1:
            print(stringToPrint)

    def positionPIDCallback(self, move):
        self.movePass = move
        self.targetRate = move.theta

    def gyroCallback(self, gyro):
        pub = rospy.Publisher('/thrusterMapping',Movement)
        move = Movement()
        P = float(rospy.get_param('~P', '.01'))
        D = float(rospy.get_param('~D', '.01'))

        r = P*(self.targetRate - gyro.rate)+D*((self.targetRate - gyro.rate)-self.previousError)
        self.previousError = self.targetRate - gyro.rate

        self.printDebug('TargetRate:{:5.3f}  Gyro Rate:{:5.3f}  RateDiff:{:5.3f}'.format(
            self.targetRate, gyro.rate, self.previousError))

        move.theta = r
        if math.fabs(targetRate - gyro.rate) < 3:
                move.theta = 0
        move.x=movePass.x
        move.y=movePass.y

        pub.publish(move)

if __name__ == '__main__':
    try:
        ne = Arbitrator()
        rospy.spin()
    except rospy.ROSInterruptException: pass
