#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('lab2')
roslib.load_manifest('hovercraft')
from sensor_msgs.msg import Joy
from hovercraft.msg import Thruster
from lab2.msg import Switcher, MovementRaw, Movement
import math

statemachine=1  #for triangle different stage
first=True     #for targetAngle
counter=0     #publish times counter 
#state = 0

def listener():
	rospy.init_node('triangle')
	#rospy.Subscriber('/joyArbitrator',Switcher, callback)
	rospy.Subscriber('/hovercraft/Gyro',Gyro,gyroCallback)
	rospy.spin()
'''
def callback(switch):
	global statemachine
	global state
	
	state = switch.state
	if state == TRIANGLE:
		statemachine=1
	else:
		statemachine=0
'''		
def gyroCallback(gyro):
	global statemachine
	global first
	global counter
	pub = rospy.Publisher('/triangleOut',Movement)
	move=Movement()

	#go right	
	if statemachine == 1:
		counter=counter+1
		if counter<=10:    #the number need to be decided from experiment, may be a large number, to get triangle link length
			move.theta=0
			move.x=-1
			move.y=0
			#pub.publish(move)
	else:
		counter=0
		statemachine=2
		
	#rotate anticlockwise for 120 degree
	if statemachine ==2:
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
				statemachine=3
		
	#second edge
	if statemachine == 3:
		counter=counter+1
		if counter<=10:    #number same case
			move.theta=0
			move.x=-1
			move.y=0
			#pub.publish(move)
	else:
		counter=0
		statemachine=4

	#second rotate anticlockwise for 120 degree 
	if statemachine ==4:
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
				statemachine=5

	#third edge
	if statemachine == 5:
		counter=counter+1
		if counter<=10:        #number same case
			move.theta=0
			move.x=-1
			move.y=0
			#pub.publish(move)
	else:
		counter=0
		statemachine=1

	pub.publish(move)

if __name__ == '__main__':
        listener() 

