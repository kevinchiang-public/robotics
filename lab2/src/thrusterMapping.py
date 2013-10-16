#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('lab2')
roslib.load_manifest('hovercraft')
from lab2.msg import Movement
from hovercraft.msg import Thruster
import math
from copy import deepcopy as deep

class ThrusterMapping():
    def __init__(self):
        print "Thrusters Ready"
        self.thrust = Thruster()
        self.debug = float(rospy.get_param('~debug', '0'))
        self.liftPower = float(rospy.get_param('~liftPower', '.3'))
        rospy.Subscriber("/thrusterMapping",Movement,self.fireThrusters)

    def fireThrusters(self, move):
        theta = deep(move.theta)
        x = deep(move.x)
        y = deep(move.y)

        #Translation
        coef=0.75
        tr1x=0
        tr2x=0
        tr3x=0
        tr1y=0
        tr2y=0
        tr3y=0

        if -x>0:
            tr2x=1/0.866*(-x)*coef
            tr1x=0.5/0.866*(-x)*coef
        if -x<0:
            tr3x=1/0.866*x*coef
            tr1x=0.5/0.866*x*coef
        if y>0:
            tr2y=1*y*coef
            tr3y=1*y*coef
        if y<0:
            tr1y=1*(-y)*coef

        self.thrust.thruster1=tr1x+tr1y
        self.thrust.thruster2=tr2x+tr2y
        self.thrust.thruster3=tr3x+tr3y

        if theta >0:
            #Turn on 4
            self.thrust.thruster5 = 0
            self.thrust.thruster4 = math.fabs(theta)
            self.thrust.thruster4 = self.thrust.thruster4 if self.thrust.thruster4 < .5 else .5
        elif theta<0:
            #Turn on 5
            self.thrust.thruster4 = 0
            self.thrust.thruster5 = math.fabs(theta)
            self.thrust.thruster5 = self.thrust.thruster5 if self.thrust.thruster5 < .5 else .5
        if (self.debug == 1):
            print ("Thruster 1:%6.2f Thruster 2:%6.2f Thruster 3:%6.2f\nTheta:%6.2f  Thruster 4:%6.2f  Thruster 5:%6.2f Lift:%6.2f\nX: %6.2f Y: %6.2f" %
                   (self.thrust.thruster1, self.thrust.thruster2, self.thrust.thruster3, theta, self.thrust.thruster4, self.thrust.thruster5, self.thrust.lift,x, y))

        #Kill the thrusters if no lift is present
        if move.lift == False:
            self.thrust.thruster1 = 0
            self.thrust.thruster2 = 0
            self.thrust.thruster3 = 0
            self.thrust.thruster4 = 0
            self.thrust.thruster5 = 0
            self.thrust.lift = 0
        else:
            self.thrust.lift = self.liftPower

        pub = rospy.Publisher('/hovercraft/Thruster', Thruster)

	#if self.thrust.thruster1 > 0 or self.thrust.thruster2 > 0 or self.thrust.thruster3 > 0 or self.thrust.thruster4 > 0 or self.thrust.thruster5 > 0:	
	#	print self.thrust

        pub.publish(self.thrust)

#Entry point for ROS
if __name__ == '__main__':
    rospy.init_node('ThrusterMapping')
    try:
        ne = ThrusterMapping()
        rospy.spin()
    except rospy.ROSInterruptException: pass
