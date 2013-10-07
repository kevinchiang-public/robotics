#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('lab2')
roslib.load_manifest('hovercraft')
from sensor_msgs.msg import Joy
from hovercraft.msg import Thruster
from lab2.msg import Switcher, MovementRaw, Movement

state = 0
liftOn  = False
currentThrust = Thruster()
def thrustersOn(thrust):
	if thrust.lift > 0:
		return True
	elif thrust.thruster1 > 0:
		return True
	elif thrust.thruster2 > 0:
		return True
	elif thrust.thruster3 > 0:
		return True
	elif thrust.thruster4 > 0:
		return True
	elif thrust.thruster5 > 0:
		return True
	else:
		return False

def callback(switcher):
	global state
	global liftOn
	global currentThrust
	'''
	JOYSTICK = 0
	TRIANGLE = 1
	REACTIVE = 2
	'''

	state = switcher.state
	liftOn  = switcher.lift
	publisher = rospy.Publisher('/arbitratorThruster',Thruster)
	thrust = Thruster()
	if not liftOn and thrustersOn(currentThrust):
		offPub = rospy.Publisher('/hovercraft/Thruster',Thruster)
		thrust.lift = 0 
		thrust.thruster1=0
		thrust.thruster2=0
		thrust.thruster3=0
		thrust.thruster4=0
		thrust.thruster5=0
		offPub.publish(thrust)
	elif liftOn:
		thrust.lift = .3	
		thrust.thruster1=0
		thrust.thruster2=0
		thrust.thruster3=0
		thrust.thruster4=0
		thrust.thruster5=0
	publisher.publish(thrust)
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
	if liftOn and state==JOYSTICK:
		publisher.publish(move)
		
def triangleCallback():
	pass #TODO implement this.  The logic should be similar to manualCallback

def reactiveCallback():
	pass #TODO see above

def thrusterCallback(thruster):
	global currentThrust
	currentThrust = thruster

def listener():
	rospy.init_node('arbitrator')
	rospy.Subscriber('/hovercraft/Thruster',Thruster, thrusterCallback)
	rospy.Subscriber('/joyArbitrator', Switcher, callback)
	rospy.Subscriber('/angleIntegratorOut', Movement, manualCallback)
	#rospy.Subscriber('/triangleOut', Type, triangleCallback)
	#rospy.Subscriber('/reactiveOut', Type, reactiveCallback)
	rospy.spin()

if __name__ == '__main__':
	listener()
