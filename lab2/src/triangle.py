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
initial=True

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
		print ("triangle start")
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
		print("statemachine 1")
		counter=counter+1
		if counter<=80:    #the number need to be decided from experiment, may be a large number, to get triangle link length
			if first == True:
				targetAngle=gyro.angle
				first=False
			move.theta=targetAngle
			move.x=-0.3
			move.y=0
			#pub.publish(move)
		else:
			counter=0
			first=True
			move.theta=targetAngle
			move.x=0
			move.y=0
			statemachine=2
		
	#rotate anticlockwise for 120 degree
	if statemachine ==2:
		print("statemachine 2")
		if first:
			targetAngle = gyro.angle
			first = False
			targetAngle += 120
		move.theta=targetAngle
		move.x=0
		move.y=0
		#pub.publish(move)
		if math.fabs(targetAngle-gyro.angle)<=3:
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
		print("statemachine 3")
		counter=counter+1
		if counter<=80:    #number same case
			#if first == True:
				#targetAngle=gyro.angle
				#first=False
			move.theta=targetAngle
			move.x=-0.3
			move.y=0
			#pub.publish(move)
		else:
			counter=0
			#first=True
			move.theta=targetAngle
			move.x=0
			move.y=0
			statemachine=4

	#second rotate anticlockwise for 120 degree 
	if statemachine ==4:
		print("statemachine 4")
		if first:
			targetAngle = gyro.angle
			first = False
			targetAngle += 120
		move.theta=targetAngle
		move.x=0
		move.y=0
		#pub.publish(move)
		if math.fabs(targetAngle-gyro.angle)<=3:
			counter=counter+1
			if counter>10:   #number same case
				counter=0
				first=True
				move.theta=gyro.angle
				move.x=0
				move.y=0
				statemachine=5

	#third edge
	if statemachine == 5:
		print("statemachine 5")
		counter=counter+1
		if counter<=80:        #number same case
			#if first == True:
				#targetAngle=gyro.angle
				#first=False
			move.theta=targetAngle
			move.x=-0.3
			move.y=0
			#pub.publish(move)
		else:
			counter=0
			#first=True
			statemachine=0
			initial=True
			move.theta=targetAngle
			move.x=0
			move.y=0

	pub.publish(move)

if __name__ == '__main__':
        listener() 

