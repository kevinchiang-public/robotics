#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('lab2')
from lab2.msg import Switcher

class Switch():
    def __init__(self):
        self.debug = float(rospy.get_param('~debug', '0'))
        rospy.Subscriber('/joySwitch',Switcher,self.joyCallback)
        rospy.Subscriber('/mappingSwitch',Switcher,self.mappingCallback)

        self.joySwitcher = Switcher()
        self.mappingSwitcher = Switcher()
        self.tangentSwitcher = Switcher()
        self.visualServoSwitcher = Switcher()

    def joyCallback(self,switch):
        self.joySwitcher = switch
        self.switch(self.joySwitcher)

    def mappingCallback(self,switch):
         self.mappingSwitcher = switch
         self.switch(self.mappingSwitcher)

    def tangentCallback(self,switch):
        self.tangentSwitcher = switch
        self.switch(self.tangentSwitcher)

    def visualServoCallback(self, switch):
        self.visualServoSwitcher = switch
        self.switch(self.visualServoSwitcher)

    def switch(self, switch):
        joyState = self.joySwitcher.state
        publisher = rospy.Publisher('/switchArbitrator', Switcher)
        switchOut = Switcher()
        switchOut.lift = self.joySwitcher.lift

        if joyState == 0 or joyState == 1 or joyState == 2 or joyState == 5 or joyState == 6:
            switchOut.state = self.joySwitcher.state
        elif joyState == 3:
            switchOut.state = self.mappingSwitcher.state
        elif joyState == 4:
            switchOut.state = self.tangentSwitcher.state

        publisher.publish(switchOut)
        #self.printDebug("JoyState: %6.2f, MappingState: %6.2f"%(joyState,self.mappingSwitcher.state))

    def printDebug(self,string):
        if self.debug ==1:
            print string

if __name__ == '__main__':
    rospy.init_node('Switch')
    try:
        ne = Switch()
        rospy.spin()
    except rospy.ROSInterruptException: pass
