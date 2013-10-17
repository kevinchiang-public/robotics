#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('lab2')
roslib.load_manifest('hovercraft')
from lab2.msg import Range

class Mapping():
    def __init__(self):
        self.debug = float(rospy.get_param('~debug','0'))
        self.currentAngle = 0
        rospy.Subscriber('/spinOut', spinInfo, self.spinCallback)
        self.spinOn = False

    def spinCallback(self,spin):
        pass

        

if __name__ == '__main__':
    rospy.init_node('Mapping')
    try:
        ne = Mapping()
        rospy.spin()
    except rospy.ROSInterruptException: pass
