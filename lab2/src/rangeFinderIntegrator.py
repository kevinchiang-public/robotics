#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('hovercraft')
roslib.load_manifest('lab2')
from hoverboard.msg import ADCRaw
import math

class RangeFinderIntegrator():
    def __init__(self):
	print "RangeFinderIntegrator initialized"
        self.debug = float(rospy.get_param('~debug', 0))
        rospy.Subscriber('/hoverboard/ADCRaw', ADCRaw, self.integrateRawValues) #Probably wrong

    def integrateRawValues(self, ADCRaw):
	#print "integrateRawValue Callback"
        #Stuff converting our raw values into range information
	leftIR = ADCRaw.adc5_1
	rightIR= ADCRaw.adc5_0

	print ("Left: %6.2f   Right: %6.2f\n"%(leftIR,rightIR))

if __name__ == '__main__':
    rospy.init_node('RangeFinderIntegrator')
    try:
        ne = RangeFinderIntegrator()
        rospy.spin()
    except rospy.ROSInterruptException: pass
