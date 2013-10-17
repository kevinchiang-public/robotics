#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('lab2')
roslib.load_manifest('hovercraft')

from hovercraft.msg import Gyro
from lab2.msg import Movement
from lab2.msg import SpinnerRang

class Spinner():
    def __init__(self):
        self.debug = float(rospy.get_param('~debug','0'))
        rospy.Subscriber('/hovercraft/Gyro', Gyro, self.gyroCallback)
        rospy.Subscriber('/rangeInfo',Range, self.rangeCallback)
        self.currentAngle = 0
        self.initialAngle = 0
        self.spin = False
        self.spinnerRangeMsg = SpinnerRange()
        self.First = True

    def gyroCallback(self,gyro):
        if self.First:
            self.initialAngle = gyro.angle
            self.First = False
        self.printDebug('Initial Angle: %6.2f TargetAngle: %6.2f Gyro Angle: %6.2f'%(self.initialAngle, targetAngle, gyro.angle))
        if gyro.angle > targetAngle:
            move = Movement()
            move.x = 0
            move.y = 0
            move.theta = -10
            self.IR = []
            publisher = rospy.Publisher('/spinOut', Movement)

    def rangeCallback(self, ranges):
        leftIR = ranges.leftDistanceCM
        rightIR= ranges.rightDistanceCM
        self.IR.append((leftIR,rightIR))

    def printDebug(self, string):
        if self.debug==1:
            print string

if __name__ == '__main__':
    rospy.init_node('Spinner')
    try:
        ne = Spinner()
        rospy.spin()
    except rospy.ROSInterruptException: pass
