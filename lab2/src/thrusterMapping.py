#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('lab2')
from lab2.msg import Movement
from hovercraft.msg import Thruster
import math

class ThrusterMapping():
    def __init__self(self):
        self.thrust = thruster()
        self.debug = rospy.get_param('~debug', '0')
        rospy.init_node('thrusterMapping')
        rospy.Subscriber("/thrusterMapping",Movement,self.fireThrusters)
        #rospy.Subscriber("/arbitratorThruster",Thruster,arbitratorCallback)

    #def arbitratorCallback(thrust):
        #self._thruster = thrust

    def fireThrusters(self, move):
        theta = move.theta
        x = move.x
        y = move.y

        if theta >0:
            #Turn on 4
            self.thrust.thruster5 = 0
            self.thrust.thruster4 = math.fabs(theta)
            self.thrust.thruster4 = thrust.thruster4 if thrust.thruster4 < .5 else .5
        elif theta<0:
            #Turn on 5
            self.thrust.thruster4 = 0
            self.thrust.thruster5 = math.fabs(theta)
            self.thrust.thruster5 = thrust.thruster5 if thrust.thruster5 < .5 else .5

        if (self.debug == 1):
            print ("Theta:%6.2f  Thruster 4:%6.2f  Thruster 5:%6.2f" %
                   (theta, self.thrust.thruster4, self.thrust.thruster5))

        pub = rospy.Publisher('/hovercraft/Thruster', Thruster)
        pub.publish(self.thrust)

#Entry point for ROS
if __name__ == '__main__':
    rospy.init_node('ThrusterMapping')
    try:
        ne = ThrusterMapping()
        rospy.spin()
    except rospy.ROSInterruptException: pass
