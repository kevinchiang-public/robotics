#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('lab2')
from lab2.msg import Movement
import math

class ReactiveControl():
    def __init__(self):
        self.debug = float(rospy.get_param('~debug', '0'))
        rospy.Subscriber('/rangeInfo', RangeInfo, self.reactToRange)

    def reactToRange(self,rangeInfo):
        publisher = rospy.Publisher('/reactiveOut', Movement)

        publisher.publish(moveOut)

        #Prints all information related to the ReactiveControl if need be

if __name__ == '__main__':
    rospy.init_node('ReactiveControl')
    try:
        ne = ReactiveControl()
        rospy.spin()
    except rospy.ROSInterruptException: pass
