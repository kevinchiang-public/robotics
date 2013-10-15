#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('hovercraft')
roslib.load_manifest('lab2')
from hoverboard.msg import ADCRaw
import math

class RangeFinderIntegrator.py:
    def __init__(self):
        self.debug = float(rospy.get_param('~debug', 0))
        rospy.Subscriber('/hoverboard/ADCRaw', ADCRaw, self.integrateRawValues) #Probably wrong

    def integrateRawValues(self, ADCRaw):
        #Stuff converting our raw values into range information


if __name__ == '__main__':
    rospy.init_node('RangeFinderIntegrator')
    try:
        ne = RangeFinderIntegrator()
        rospy.spin()
    except rospy.ROSInterruptException: pass
