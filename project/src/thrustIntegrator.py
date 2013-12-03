#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('hovercraft')
from hovercraft.msg import Thruster

class Integrator():
    def __init__(self):
        self.publisher = rospy.Publisher('/hovercraft/Thruster', Thruster)
        self.thrust = Thruster()
        self.thrust.lift=0
        self.thrust.thruster1=0
        self.thrust.thruster2=0
        self.thrust.thruster3=0
        self.thrust.thruster4=0
        self.thrust.thruster5=0
        self.thrust.thruster6=0

        rospy.Subscriber('/thrust/Integrator/launch',Thruster,self.launchCallback)
        rospy.Subscriber('/thrust/Integrator/move',Thruster,self.moveCallback)

    def launchCallback(launch):
        self.thrust.thruster6 = launch.thruster6
        self.publisher.publish(self.thrust)

    def moveCallback(move):
        self.thrust.lift = move.lift
        self.thrust.thruster1 = move.thruster1
        self.thrust.thruster2 = move.thruster2
        self.thrust.thruster3 = move.thruster3
        self.thrust.thruster4 = move.thruster4
        self.thrust.thruster5 = move.thruster5
        self.publisher.publish(self.thrust)

if __name__ == '__main__':
        rospy.init_node('ThrustIntegrator')
        try:
            ne = Integrator()
            rospy.spin()
        except rospy.ROSInterruptException: pass
