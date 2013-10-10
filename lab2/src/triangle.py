#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('lab2')
roslib.load_manifest('hovercraft')
from sensor_msgs.msg import Joy
from hovercraft.msg import Thruster
from lab2.msg import Switcher, MovementRaw, Movement
import math

statemachine=0  #for triangle 
first=True     #for targetAngle
counter=0     #check the rotation 
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

def listener():
	rospy.init_node('triangle')
	rospy.Subscriber('/joyArbitrator',Switcher, callback)
	rospy.Subscriber('/hovercraft/Gyro',Gyro,gyroCallback)
	rospy.spin()

def callback(switch):
	global statemachine
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
	
	publisher = rospy.Publisher('/triangleThruster',Thruster)
		
	if state == TRIANGLE:
		thrust = Thruster()
		if not liftOn and thrustersOn(currentThrust):
			thrust.lift = 0 
			thrust.thruster1=0
			thrust.thruster2=0
			thrust.thruster3=0
			thrust.thruster4=0
			thrust.thruster5=0
			statemachine=0
		elif liftOn:
			thrust.lift = .3	
			thrust.thruster1=0
			thrust.thruster2=0
			thrust.thruster3=0
			thrust.thruster4=0
			thrust.thruster5=0
			statemachine=1
	publisher.publish(thrust)
	
def gyroCallback(gyro):
	global statemachine
	global currentThrust
	publisher = rospy.Publisher('/triangleThruster',Thruster)
	pub = rospy.Publisher('/triangleOut',Movement)
	move=Movement()
	#go right	
	if statemachine == 1:
		thrust = Thruster()
		for x in xrange(1, 11):
			thrust.thruster1=0.25
			thrust.thruster2=0.5
		statemachine=2
		thrust.thruster1=0
		thrust.thruster2=0
		statemachine=2
	#rotate anticlockwise for 120 degree
	if statemachine ==2:
		move=Movement()
		if first:
			targetAngle = gyro.angle
			first = False
			targetAngle += 120
		P = float(rospy.get_param('~P', '.01'))
		D = float(rospy.get_param('~D', '.01'))
		r = P*(targetAngle - gyro.angle)+D*((targetAngle - gyro.angle)-previousError)
		#Derivative part
		previousError = targetAngle - gyro.angle
		move.theta = r 
		if math.fabs(targetAngle - gyro.angle) < 3:
			move.theta = 0
		move.x=0
		move.y=0
		
		for x in xrange(1,11):
			if move.theta=0
				counter += 1
		if counter == 10
			statemachine=3
			first=True
			counter=0
			move.theta = 0
	#second edge
	if statemachine == 3:
		thrust = Thruster()
		for x in xrange(1, 11):
			thrust.thruster1=0.25
			thrust.thruster2=0.5
		statemachine=4
		thrust.thruster1=0
		thrust.thruster2=0

	#second rotate anticlockwise for 120 degree 
	if statemachine ==4:
		move=Movement()
		if first:
			targetAngle = gyro.angle
			first = False
			targetAngle += 120
		P = float(rospy.get_param('~P', '.01'))
		D = float(rospy.get_param('~D', '.01'))
		r = P*(targetAngle - gyro.angle)+D*((targetAngle - gyro.angle)-previousError)
		#Derivative part
		previousError = targetAngle - gyro.angle
		move.theta = r 
		if math.fabs(targetAngle - gyro.angle) < 3:
			move.theta = 0
		move.x=0
		move.y=0
		
		for x in xrange(1,11):
			if move.theta=0
				counter += 1
		if counter == 10
			statemachine=5
			first=True
			counter=0
			move.theta = 0

	#third edge
	if statemachine == 5:
		thrust = Thruster()
		for x in xrange(1, 11):
			thrust.thruster1=0.25
			thrust.thruster2=0.5
		statemachine=0
		thrust.thruster1=0
		thrust.thruster2=0

	publisher.publish(thrust)
	pub.publish(move)


if __name__ == '__main__':
        listener() 
