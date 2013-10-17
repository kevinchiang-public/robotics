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
        self.initialAngle = 0 #Updated in getCurrentAngle
        self.currentAngle = 0

        rospy.Subscriber('/hovercraft/Gyro', Gyro, self.getCurrentAngle)
        rospy.Subscriber('/rangeInfo', Range, self.getCurrentRange)
        rospy.Subscriber('/joyArbitrator', Switcher, self.switchMode)

    def getCurrentAngle(self, gyroInfo):
        self.currentAngle = gyroInfo.angle

    def getCurrentRange(self,range):
        moveOut = Movement()

    #Lets the mapper know to start mapping
    def switchMode(self,switch):
        if switch.state == 3 and stateToggle == False:
            self.mode = 'Find'
            self.stateToggle = True
        elif switch.state != 3:
            self.mode = 'Stop'
            self.stateToggle = False



if __name__ == '__main__':
    rospy.init_node('Mapping')
    try:
        ne = Mapping()
        rospy.spin()
    except rospy.ROSInterruptException: pass
