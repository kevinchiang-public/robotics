#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('lab2')
roslib.load_manifest('hovercraft')
from lab2.msg import Range
from hovercraft.msg import Gyro

#Follow closest object like a lost puppy
class Mapping():
    def __init__(self):
        self.debug = float(rospy.get_param('~debug','0'))
        self.findRate = float(rospy.get_param('~findRate','0'))
        self.mode = 'Stop' #Also: 'Find' or 'Follow'
        self.stateToggle = False #Switch hasn't been switched on and off yet
        self.setInitialAngle = False #Set to true whenver find mode is started

        self.initialAngle = 0 #Updated in getCurrentAngle
        self.currentAngle = 0
        self.targetAngle = 0

        #Essentially the target angle
        self.objectDistance = float("inf")
        self.objectAngle = 0

        rospy.Subscriber('/hovercraft/Gyro', Gyro, self.getCurrentAngle)
        rospy.Subscriber('/rangeInfo', Range, self.reactToRange)
        rospy.Subscriber('/joyArbitrator', Switcher, self.switchMode)

    def getCurrentAngle(self, gyroInfo):
        self.currentAngle = gyroInfo.angle
        if self.setInitialAngle = True
            self.initialAngle = self.currentAngle
            self.setInitialAngle = False

    def reactToRange(self,range):
        moveOut = Movement()
        if self.mode == 'Find':
            #Find the target angle
            if (math.floor(currentAngle) - math.floor(initialAngle)) > 0:
                self.mode == 'Follow'

        elif self.mode == 'Follow':
            #If both IR ranges are valid
            moveOut.theta = self.targetAngle
            moveOut.y = 1
            moveOut.x = 0
            #elif right IR is shitty

            if bla < bla: #we need to do a find#
                self.setInitialAngle = True
            pass
        else:
            moveOut.theta = 0
            moveOut.x = 0
            moveOut.y = 0

    #Lets the mapper know to start mapping
    def switchMode(self,switch):
        if switch.state == 3 and stateToggle == False:
            self.mode = 'Find'
            self.stateToggle = True
            self.setInitialAngle = True
        elif switch.state != 3:
            self.mode = 'Stop'
            self.setInitialAngle = False
            self.stateToggle = False



if __name__ == '__main__':
    rospy.init_node('Mapping')
    try:
        ne = Mapping()
        rospy.spin()
    except rospy.ROSInterruptException: pass
