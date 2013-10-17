#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('hovercraft')
from hovercraft.msg import Gyro
from lab2.msg import Switcher, Movement, Range, irData
import math
class Mapping():
    def __init__(self):
        self.debug = float(rospy.get_param('~debug','0'))
        rospy.Subscriber('/rangeInfo', Range, self.rangeCallback)
        rospy.Subscriber('/hovercraft/Gyro', Gyro, self.gyroCallback)
        rospy.Subscriber('/switchArbitrator', Switcher, self.switcherCallback)
        self.hasBegun = False
        self.currentAngle = 0
        self.initialAngle = 0
        self.gyroFirst = True
        self.currentDistanceL = 0
        self.currentDistanceR = 0
        self.move = Movement()
        self.leftDistance = [None]*360
        self.rightDistance= [None]*360
        

    def rangeCallback(self,ranges):
        self.currentDistanceL = ranges.leftDistanceCM
        self.currentDistanceR = ranges.rightDistanceCM

        #The index is the floor of the difference between current and self
        index = math.floor(self.currentAngle - self.initialAngle)
        
        #This will set the left distances, or average the two if there are already values in the            array at that index
        if self.leftDistance[index] is not None:
            self.leftDistance[index] = (self.leftDistance[index]+self.currentDistanceL)/2.0
        else:
            self.leftDistance[index] = self.currentDistanceL

        #Same as above, but for right distances
        if self.rightDistance[index] is not None:
            self.rightDistance[index] = (self.rightDistance[index]+self.currentDistanceR)/2.0
        else:
            self.rightDistance[index]= self.currentDistanceR

    def gyroCallback(self,gyro):
        self.currentAngle = gyro.angle
        if self.gyroFirst:
            self.initialAngle = gyro.angle
            self.gyroFirst = False

        if self.hasBegun and self.currentAngle < self.initialAngle + 360:
            self.move.theta = float(rospy.get_param('~rotationalUnit','2'))
            self.move.modType= 'Add'
            self.move.x =0
            self.move.y =0
            publisher = rospy.Publisher('/mappingOut',Movement)
            publisher.publish(self.move)

        #Publish to tangent bug once it completes a revolution
        elif self.hasBegun and self.currentAngle >= self.initialAngle + 360:
            publisher = rospy.Publisher('/irData',irData)
            rangeData = irData()
            rangeData.leftDistance = self.leftDistance
            rangeData.rightDistance= self.rightDistance
            rangeData.initialAngle = self.initialAngle

            publisher.publish(rangeData)

            self.printDebug(self.leftDistance)
            self.printDebug(self.rightDistance)

            #It has ended so set hasBegun to false then "zero out" the existing rangeData
            self.hasBegun = False
            self.leftDistance = [None]*360
            self.rightDistance= [None]*360

    def switcherCallback(self, switch):
        if switch.state == 3 and not self.hasBegun:
            self.initialAngle = self.currentAngle
            self.hasBegun = True
            self.gyroFirst = True

    def printDebug(self, string):
        if debug ==1:
            print string

if __name__ == '__main__':
    rospy.init_node('Mapper')
    try:
        ne = Mapping()
        rospy.spin()
    except rospy.ROSInterruptException: pass
