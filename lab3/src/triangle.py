#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('lab2')
roslib.load_manifest('hovercraft')
from hovercraft.msg import Gyro
from sensor_msgs.msg import Joy
from hovercraft.msg import Thruster
from lab2.msg import Switcher, MovementRaw, Movement
import math

statemachine=0  #for triangle different stage
first=True     #for targetAngle
counter=0     #publish times counter 
targetAngle=0
state = 0
initial=True  #triangle loop flag

def printDebug(string):
	debug = float(rospy.get_param('~debug','0'))
	if debug == 1:
		print string

def listener():
	rospy.init_node('triangle')
	rospy.Subscriber('/joyArbitrator',Switcher, callback)
	rospy.Subscriber('/hovercraft/Gyro',Gyro,gyroCallback)
	rospy.spin()

def callback(switch):
	global statemachine
	global state
	global initial
	
	state = switch.state
	if state == 1 and initial == True:
		printDebug ("triangle start")
		statemachine=1
		initial=False
	#else:
		#statemachine=0
		
def gyroCallback(gyro):
	global statemachine
	global first
	global counter
	global targetAngle
	global initial
	pub = rospy.Publisher('/triangleOut',Movement)
	move=Movement()

	#go right	
	if statemachine == 1:
		printDebug("statemachine 1")
		counter=counter+1
		if counter<=80:    #the number need to be decided from experiment, may be a large number, to get triangle link length
			if first == True:
				targetAngle=gyro.angle
				first=False
			move.theta=targetAngle
			move.x=-0.5
			move.y=0
			#pub.publish(move)
		else:
			counter=0
			first=True
			move.theta=targetAngle
			move.x=0
			move.y=0
			statemachine=12

	#intermediate state
	if statemachine == 12:
		counter=counter+1
		move.theta=targetAngle
		move.x=0.2
		move.y=0
		if counter>10:
			counter=0
			statemachine=2
		
	#rotate anticlockwise for 120 degree
	if statemachine == 2:
		printDebug("statemachine 2")
		if first:
			targetAngle = gyro.angle
			first = False
			targetAngle += 120
		move.theta=targetAngle
		move.x=0
		move.y=0
		#pub.publish(move)
		if math.fabs(targetAngle-gyro.angle)<=10:
			counter=counter+1
			if counter>10:    #the number need to be decided from experiment, may be a large number, to make sure the angle has been achived
				counter=0
				first=True
				move.theta=targetAngle
				move.x=0
				move.y=0
				statemachine=3
		
	#second edge
	if statemachine == 3:
		printDebug("statemachine 3")
		counter=counter+1
		if counter<=80:    #number same case
			if first == True:
				targetAngle=gyro.angle
				first=False
			move.theta=targetAngle
			move.x=-0.5
			move.y=0
			#pub.publish(move)
		else:
			counter=0
			first=True
			move.theta=targetAngle
			move.x=0
			move.y=0
			statemachine=34

	#intermediate state
	if statemachine == 34:
		counter=counter+1
		move.theta=targetAngle
		move.x=0.2
		move.y=0
		if counter>10:
			counter=0
			statemachine=4

	#second rotate anticlockwise for 120 degree 
	if statemachine ==4:
		printDebug("statemachine 4")
		if first:
			targetAngle = gyro.angle
			first = False
			targetAngle += 120
		move.theta=targetAngle
		move.x=0
		move.y=0
		#pub.publish(move)
		if math.fabs(targetAngle-gyro.angle)<=10:
			counter=counter+1
			if counter>10:   #number same case
				counter=0
				first=True
				move.theta=targetAngle
				move.x=0
				move.y=0
				statemachine=5

	#third edge
	if statemachine == 5:
		printDebug("statemachine 5")
		counter=counter+1
		if counter<=80:        #number same case
			if first == True:
				targetAngle=gyro.angle
				first=False
			move.theta=targetAngle
			move.x=-0.5
			move.y=0
			#pub.publish(move)
		else:
			counter=0
			first=True
			statemachine=56
			move.theta=targetAngle
			move.x=0
			move.y=0

	#intermediate state
	if statemachine == 56:
		printDebug("statemachine 56")
		counter=counter+1
		move.theta=targetAngle
		move.x=0.2
		move.y=0
		if counter>10:
			counter=0
			statemachine=60

	if statemachine == 60:
		printDebug("statemachine 60")
		counter=counter+1
		move.theta=gyro.angle
		move.x=0
		move.y=0
		if counter>1000:
			counter=0
			statemachine=0
			initial=True

	pub.publish(move)

if __name__ == '__main__':
        listener() 

