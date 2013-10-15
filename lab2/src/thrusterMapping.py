#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('lab2')
roslib.load_manifest('hovercraft')
from lab2.msg import Movement
from hovercraft.msg import Thruster
import math

class ThrusterMapping():
    def __init__(self):
        self.thrust = Thruster()
        self.debug = float(rospy.get_param('~debug', '0'))
	print "Thruster Mapping Initialized"
        rospy.Subscriber("/thrusterMapping",Movement,self.fireThrusters)
        #No longer used::rospy.Subscriber("/arbitratorThruster",Thruster,self.arbitratorCallback)
	rospy.spin()
	'''
    def arbitratorCallback(self,thrust):
        self.thrust = thrust
	'''
    def fireThrusters(self, move):
        theta = move.theta
        x = move.x
        y = move.y

        #initate lift
        self.thrust.lift=.3

        #Translation
        coef=0.5
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

	print('Theta in thruster: ' + str(theta))
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
            print ("Thruster 1:%6.2f Thruster 2:%6.2f Thruster 3:%6.2f\nTheta:%6.2f  Thruster 4:%6.2f  Thruster 5:%6.2f Lift:%6.2f\n" %
                   (self.thrust.thruster1, self.thrust.thruster2, self.thrust.thruster3, theta, self.thrust.thruster4, self.thrust.thruster5, self.thrust.lift))

        pub = rospy.Publisher('/hovercraft/Thruster', Thruster)
        pub.publish(self.thrust)

#Entry point for ROS
if __name__ == '__main__':
    rospy.init_node('ThrusterMapping')
    try:
        ne = ThrusterMapping()
    except rospy.ROSInterruptException: pass
