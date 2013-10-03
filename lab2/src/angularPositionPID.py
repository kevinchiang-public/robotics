#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('lab2')
roslib.load_manifest('hovercraft')
from hovercraft.msg import Gyro
from sensor_msgs.msg import Joy
from lab2.msg import Movement

DEBUG = True
xDepressed = False
bDepressed = False
targetAngle= 0
first = True
previousError=0
def listener():
	rospy.init_node('AngularPositionPID')
	rospy.Subscriber('/hovercraft/Gyro',Gyro,gyroCallback)
	rospy.Subscriber('/joy',Joy,joyCallback)
	rospy.spin()

def gyroCallback(gyro):
	global targetAngle
	global previousError
	global first
	pub = rospy.Publisher('/thrusterMapping',Movement)
	move = Movement()
	if first:
		targetAngle = gyro.angle
		first = False
	P = float(rospy.get_param('~P', '.01'))
	D = float(rospy.get_param('~D', '.01'))
	print P
	r = P*(targetAngle - gyro.angle)+D*((targetAngle - gyro.angle)-previousError)
	print "TargetAngle:",targetAngle,"\tGyro Angle:",gyro.angle,"\tr:",r
	#TODO Derivative part
	previousError = targetAngle - gyro.angle
	move.theta = r
	move.x=0
	move.y=0
	pub.publish(move)
def joyCallback(joy):
	global xDepressed
	global bDepressed
	global targetAngle

	#THESE ARE FLIPPED ON PURPOSE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	xButton = joy.buttons[1]
	bButton = joy.buttons[2]

	if bButton == 1 and not bDepressed:
		bDepressed = True
		print "B button pushed"
		targetAngle += 90
	elif xButton == 1 and not xDepressed:
		xDepressed = True
		print "X button pushed"
		targetAngle += -90
	if bButton == 0 and bDepressed:
		bDepressed = False
	if xButton == 0 and xDepressed:
		xDepressed = False

if __name__ == '__main__':
	listener() 
