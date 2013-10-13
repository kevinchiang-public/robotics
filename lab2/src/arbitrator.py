#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('lab2')

from sensor_msgs.msg import Joy
from hovercraft.msg import Thruster
from lab2.msg import Switcher, MovementRaw, Movement


class Arbitrator():
    def __init__(self):
        self.debug = float(rospy.get_param('~debug', '0'))
        self.movement = Movement()
        self.liftOn = False
        self.state = 'Manual'
        rospy.init_node('Arbitrator')
        rospy.Subscriber('/joyArbitrator', Switcher, self.joyCallback)
        rospy.Subscriber('/angleIntegratorOut', Movement, self.manualJoyControl)
        #rospy.Subscriber('/triangleOut', Movement?, triangleCallback)
        #rospy.Subscriber('/reactiveOut', Movement?, reactiveCallback)

    def joyCallback(self, switch):
	print self.debug
	self.liftOn = switch.lift
        if (switch.state == 0):
            self.state = 'Manual'
        elif (switch.state == 1):
            self.state = 'Triangle'
        elif (switch.state == 2):
            self.state = 'Reactive'

        publisher = rospy.Publisher('/arbitratorOut', Movement)
        if not self.liftOn:
	    tempMove=Movement()
	    tempMove.theta=0
	    tempMove.x=0
	    tempMove.y=0
	    if self.debug == 1: 
	        print('In arbitrator: X:%3.2f  Y:%3.2f  Theta:%3.2f  State:%10s' %
		     (tempMove.x, tempMove.y, tempMove.theta, self.state))
            publisher.publish(tempMove)
        elif self.liftOn:
	    if self.debug == 1:
	        print('In arbitrator: X:%3.2f  Y:%3.2f  Theta:%3.2f  State:%10s' %
		     (tempMove.x, tempMove.y, tempMove.theta, self.state))
	  
            publisher.publish(self.movement)

    def manualJoyControl(self, move):
        if self.state is 'Manual':
            self.movement = move

    def triangleCallback(self, move):
        if self.state is 'Triangle':
            self.movement = move

    def reactiveCallback(self, move):
        if self.state is 'Reactive':
            self.movement = move

if __name__ == '__main__':
    try:
        ne = Arbitrator()
        rospy.spin()
    except rospy.ROSInterruptException: pass
