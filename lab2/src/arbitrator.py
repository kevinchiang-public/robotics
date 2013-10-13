#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('lab2')

from sensor_msgs.msg import Joy
from hovercraft.msg import Thruster
from lab2.msg import Switcher, MovementRaw, Movement


class Arbitrator():
    def __init__(self):
        self.debug = rospy.get_param('~debug', '0')
        self.movement = Movement()
        self.liftOn = False
        self.state = 'Manual'
        rospy.init_node('Arbitrator')
        rospy.Subscriber('/joyArbitrator', Switcher, self.joyCallback)
        rospy.Subscriber('/angleIntegratorOut', Movement, self.manualJoyControl)
        #rospy.Subscriber('/triangleOut', Movement?, triangleCallback)
        #rospy.Subscriber('/reactiveOut', Movement?, reactiveCallback)

    def joyCallback(self, switch):
        if (switch.state == 0):
            self.state = 'Manual'
        elif (switch.state == 1):
            self.state = 'Triangle'
        elif (switch.state == 2):
            self.state = 'Reactive'

        publisher = rospy.Publisher('/arbitratorOut', Movement)
        if not liftOn:
            self.liftOn = true
            publisher.publish(Movement(0,0,0))
        elif liftOn:
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
