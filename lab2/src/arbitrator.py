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

<<<<<<< HEAD
        publisher = rospy.Publisher('/arbitratorOut', Movement)
        if not liftOn:
            self.liftOn = true
            publisher.publish(Movement(0,0,0))
        elif liftOn:
            publisher.publish(self.movement)
=======
	state = switcher.state
	liftOn  = switcher.lift
	'''
	publisher = rospy.Publisher('/arbitratorThruster',Thruster)
	thrust = Thruster()
	if not liftOn:
		offPub = rospy.Publisher('/hovercraft/Thruster',Thruster)
		thrust.lift = 0 
		thrust.thruster1=0
		thrust.thruster2=0
		thrust.thruster3=0
		thrust.thruster4=0
		thrust.thruster5=0
		offPub.publish(thrust)
	elif liftOn:
		thrust.lift = .35	
		thrust.thruster1=0
		thrust.thruster2=0
		thrust.thruster3=0
		thrust.thruster4=0
		thrust.thruster5=0
	publisher.publish(thrust)
	'''
	'''
	if liftOn:
		if state == JOYSTICK:
			pass #TODO implement what happens during joystick control
		elif state == TRIANGLE:
			pass #TODO see above
		elif state == REACTIVE:
			pass #TODO see above
	'''
	
def manualCallback(move):
	global state
	publisher = rospy.Publisher('/arbitratorOut',Movement)
	JOYSTICK = 0
	TRIANGLE = 1
	REACTIVE = 2
	if liftOn and state==JOYSTICK:
		publisher.publish(move)
	elif not liftOn:
		off = Movement()
		off.theta = 0
		off.x=0
		off.y=0
		publisher.publish(off)
		
def triangleCallback():
	pass #TODO implement this.  The logic should be similar to manualCallback
>>>>>>> f206ec4eb970de984e84563cc89f6ad546108952

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
