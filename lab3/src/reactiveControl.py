#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('lab2')
from lab2.msg import Movement, Range
import math

class ReactiveControl():
    def __init__(self):
        self.debug = float(rospy.get_param('~debug', '0'))
        rospy.Subscriber('/rangeInfo', Range, self.reactToRange)
        self.targetDistance=20
        self.previousError=0
        self.P = float(rospy.get_param('~P','1'))
        self.D = float(rospy.get_param('~D','0'))
        self.move = Movement()


    def reactToRange(self,rangeInfo):
        left = rangeInfo.leftDistanceCM
        right= rangeInfo.rightDistanceCM
        targetDistance = self.targetDistance
        currentDistance = (left+right)/2.0

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

        publisher = rospy.Publisher('/reactiveOut', Movement)
        self.move.theta = 0
        self.move.y = -r
        self.move.x = 0
        self.move.modType = 'Add'
        publisher.publish(self.move)

        #Prints all information related to the ReactiveControl if need be
    def debugPrint(self,string):
        if self.debug == 1:
            print string


if __name__ == '__main__':
    rospy.init_node('ReactiveControl')
    try:
        ne = ReactiveControl()
        rospy.spin()
    except rospy.ROSInterruptException: pass
