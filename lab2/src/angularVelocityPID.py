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
        #Setup params from launch file
        self.debug = float(rospy.get_param('~debug', '0'))
        self.P = float(rospy.get_param('~P', '.01'))
        self.D = float(rospy.get_param('~D', '.01'))

        self.previousError = 0

        #Initialize lift to false to make sure no thrusters fire on start
        self.movePass = Movement()
        self.movePass.lift = False

        rospy.Subscriber('/hovercraft/Gyro', Gyro, self.gyroCallback)
        rospy.Subscriber('/angularPositionOut', Movement, self.positionPIDCallback)

    def debugPrint(self, stringToPrint):
        if self.debug == 1:
            print(stringToPrint)

    def positionPIDCallback(self, move):
        self.movePass = move

    def gyroCallback(self, gyro):
        targetRate = self.movePass.theta

        #Proportional and Derivative computations
        r = self.P*(targetRate - gyro.rate)
        r = r + self.D*((targetRate - gyro.rate)-self.previousError)
        self.previousError = targetRate - gyro.rate

        self.debugPrint('VelPID: TargetRate:{:5.3f}  Gyro Rate:{:5.3f}  '
                        'RateDiff:{:5.3f}'.format(targetRate, gyro.rate,
                                                  self.previousError))
        #Only reset r if the hovercraft wants to go faster in the offending direction
        if gyro.rate < -355 and r < 0:
            debugPrint("Gyro Limiter Engaged -")
            r = 0
        elif gyro.rate > 620 and r > 0:
            debugPrint("Gyro Limiter Engaged +")
            r = 0

        #Ship message off to thrusterMapping
        pub = rospy.Publisher('/thrusterMapping',Movement)
        move = Movement()
        move.theta = r
        move.x = self.movePass.x
        move.y = self.movePass.y
        move.lift = self.movePass.lift
        pub.publish(move)

if __name__ == '__main__':
    rospy.init_node('AngularVelocityPID')
    try:
        ne = AngularVelocityPID()
        rospy.spin()
    except rospy.ROSInterruptException: pass
