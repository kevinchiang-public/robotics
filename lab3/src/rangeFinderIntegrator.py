#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('hovercraft')
roslib.load_manifest('lab2')
from hoverboard.msg import ADCRaw
from lab2.msg import Range
import math

class RangeFinderIntegrator():
    def __init__(self):
        self.debug = float(rospy.get_param('~debug', '1'))
        rospy.Subscriber('/hoverboard/ADCRaw', ADCRaw, self.integrateRawValues) #Probably wrong

    def integrateRawValues(self, ADCRaw):
        #Grab data from IR sensor
        leftIR = ADCRaw.adc5_1
        rightIR= ADCRaw.adc5_0

        #Calculate actual distance using interpolated polynomial
        leftDistanceCM = self.calcLeftDistance(leftIR)
        rightDistanceCM = self.calcRightDistance(rightIR)

        #Publish data to reactive node
        publisher = rospy.Publisher('/rangeInfo', Range)
        range = Range()
        range.leftDistanceCM = leftDistanceCM
        range.rightDistanceCM = rightDistanceCM
        self.debugPrint("Left: {:6.2f}   Right: {:6.2f}".format(leftDistanceCM,rightDistanceCM))
        publisher.publish(range)

    def calcLeftDistance(self, rawValue):
        #Bring the voltage to a more reasonable value (to avoid FPE)
        rawValue = rawValue / 1000.0
        returnVal = 23.346 * math.pow(rawValue, 4) - 166.99 * math.pow(rawValue, 3)
        returnVal = returnVal + 444.88*math.pow(rawValue,2) - 549.14 * rawValue + 290.59
        return returnVal

    def calcRightDistance(self, rawValue):
        #Bring the voltage to a more reasonable value (to avoid FPE)
        rawValue = rawValue / 1000.0
        returnVal = 25.157 * math.pow(rawValue, 4) - 175.7 * math.pow(rawValue, 3)
        returnVal = returnVal + 450.38*math.pow(rawValue,2) - 525.3 * rawValue + 262.02
        return returnVal

    def debugPrint(self, string):
        if self.debug == 1:
            print string


if __name__ == '__main__':
    rospy.init_node('RangeFinderIntegrator')
    try:
        ne = RangeFinderIntegrator()
        rospy.spin()
    except rospy.ROSInterruptException: pass
