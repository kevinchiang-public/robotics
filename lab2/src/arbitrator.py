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

        #Initialize lift to false so no thrusters fire when started
        self.movement = Movement()
        self.movement.lift = False
        self.state = 'Manual'

        rospy.Subscriber('/joyArbitrator', Switcher, self.joyCallback)
        rospy.Subscriber('/angleIntegratorOut', Movement, self.manualJoyControl)
        rospy.Subscriber('/triangleOut', Movement, self.triangleCallback)
        rospy.Subscriber('/reactiveOut', Movement, self.reactiveCallback)

    def joyCallback(self, switch):
        if self.debug == 1:
            print('Mode: %s' % self.state)

        self.liftOn = switch.lift
        if (switch.state == 0):
            self.state = 'Manual'
        elif (switch.state == 1):
            self.state = 'Triangle'
        elif (switch.state == 2):
            self.state = 'Reactive'

        #Send lift state downstream (to kill thrusters if off)
        self.movement.lift = switch.lift
        publisher = rospy.Publisher('/arbitratorOut', Movement)
        publisher.publish(self.movement)

        if self.debug == 1:
            print('In arbitrator: X:%3.2f  Y:%3.2f  Theta:%3.2f  State:%10s' %
                  (self.movement.x, self.movement.y, self.movement.theta, self.state))

    #Need to track lift state in these functions manually
    #since it's set in the switcher message (essentially overriding)
    #whatever was in the packet sent from the 'controller' nodes

    #The 'mag'(magnitude) is set to 1 in the non-joystick
    #controller nodes so the rotational rate is unmodified
    #downstream.
    def manualJoyControl(self, move):
        if self.state is 'Manual':
            liftState = self.movement.lift
            self.movement = move
            self.movement.lift = liftState

    def triangleCallback(self, move):
        if self.state is 'Triangle':
            liftState = self.movement.lift
            self.movement = move
            self.movement.mag = 1
            self.movement.lift = liftState

    def reactiveCallback(self, move):
        if self.state is 'Reactive':
            liftState = self.movement.lift
            self.movement = move
            self.movement.mag = 1
            self.movement.lift = liftState

if __name__ == '__main__':
    rospy.init_node('Arbitrator')
    try:
        ne = Arbitrator()
        rospy.spin()
    except rospy.ROSInterruptException: pass
