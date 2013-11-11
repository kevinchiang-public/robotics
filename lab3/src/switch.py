#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('lab3')
from lab3.msg import Switcher
from lab3.msg import DetectionSwitcher

class Switch():
    def __init__(self):
        self.debug = float(rospy.get_param('~debug', '0'))
        rospy.Subscriber('/joySwitch',Switcher,self.joyCallback)
        rospy.Subscriber('/mappingSwitch',Switcher,self.mappingCallback)

        self.joySwitcher = Switcher()
        self.mappingSwitcher = Switcher() #Unused for now
        self.tangentSwitcher = Switcher()  #Unused for now
        self.visArbOut = DetectionSwitcher()

    def joyCallback(self,switch):
        self.joySwitcher = switch
        self.switch(self.joySwitcher)
        self.visArbOut.state = -1

    def mappingCallback(self,switch):
         self.mappingSwitcher = switch
         self.switch(self.mappingSwitcher)
         self.visArbOut.state = -1

    def switch(self, switch):
        joyState = self.joySwitcher.state
        publisher = rospy.Publisher('/switchArbitrator', Switcher)
        switchOut = Switcher()
        switchOut.lift = self.joySwitcher.lift

        if joyState == 0 or joyState == 1 or joyState == 2 or joyState == 5 or joyState == 6 or joyState == 7:
            #Do we need the camera?  If so, where should images get published to?
            if joyState == 5:  #Only publish to landmark detector node
                self.visArbOut.state = 1
            elif joyState == 6: #Only publish to ball detector node
                self.visArbOut.state = 2
            elif joyState == 7: #Currently not used. Publishes images to ball and landmark nodes
                self.visArbOut.state = 3
            else:
                self.visArbOut.state = -1
            switchOut.state = self.joySwitcher.state
        elif joyState == 3:
            switchOut.state = self.mappingSwitcher.state
        elif joyState == 4:
            switchOut.state = self.tangentSwitcher.state

        publisher.publish(switchOut)
        visArbPublisher = rospy.Publisher('/switcher/visArbitrator', DetectionSwitcher)
        visArbPublisher.publish(self.visArbOut)

    def printDebug(self,string):
        if self.debug ==1:
            print string

if __name__ == '__main__':
    rospy.init_node('Switch')
    try:
        ne = Switch()
        rospy.spin()
    except rospy.ROSInterruptException: pass
