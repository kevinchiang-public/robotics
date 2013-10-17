#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('lab2')
from lab2.msg import irData,Range,Movement,Switcher
import math

class Tangent():
    def __init__(self):
        self.debug=float(rospy.get_param('~debug','0'))
        rospy.Subscriber('/irData', irData, self.irCallback)
        rospy.Subscriber('/rangeInfo',Range, self.rangeCallback)
        self.movePublisher=rospy.Publisher('/tangentOut',Movement)
        self.move = Movement()
        self.switchPublisher=rospy.Publisher('/tangentSwitch', Switcher)
        self.switch = Switcher()
        self.leftArray=[]
        self.rightArray=[]
        self.initialAngle=0
        self.previousDistance=0

    def rangeCallback(self, ranges):
        leftIR = ranges.leftDistanceCM
        rightIR = ranges.rightDistanceCM

        #If there is an obstacle, do a map of the environment, otherwise go straight
        if leftIR < 60 or rightIR < 60:
            self.switch.state = 3
            self.switchPublisher.publish(self.switch)
        else:
            self.move.y=-1
            self.movePublisher.publish(self.move)
        
    #This happens when mapping completes a revolution and sends back array data to tangent
    def irCallback(self,ir):
        #First, give control back to tangentBug
        self.switch.state = 4
        self.switchPublisher.publish(self.switch)
        self.leftArray = ir.leftDistance
        self.rightArray= ir.rightDistance
        self.initialAngle = ir.initialAngle
        
        targetDistance=[]

        for i in range(0,360):
            if self.leftArray[i] is None and self.rightArray[i] is None:
                targetDistance.append(self.previousDistance)
            elif self.leftArray[i] is None and self.rightArray[i] is not None:
                targetDistance.append(self.rightArray[i])
            elif self.leftArray[i] is not None and self.rightArray[i] is None:
                targetDistance.append(self.leftArray[i])
            else:
                targetDistance.append((self.leftArray[i]+self.rightArray[i])/2)
            #Put the last known value into previous distance in case both are None at a particular angle
            self.previousDistance = targetDistance[-1]
        largestDiff = 0
        largestDiffIndex=0
        for i in range(0,359):
            diff = math.fabs(targetDistance[i+1]-targetDistance[i])
            if diff > largestDiff:
                largestDiff = diff
                largestDiffIndex = i

        self.move.theta = largestDiffIndex
        self.move.modType= 'Set'
        self.move.y = -1
        self.movePublisher.publish(self.move)




if __name__ == '__main__':
    rospy.init_node('tangent')
    try:
        ne = Tangent()
        rospy.spin()
    except rospy.ROSInterruptException: pass
