#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('lab2')
roslib.load_manifest('hovercraft')
from hovercraft.msg import Gyro
from sensor_msgs.msg import Joy
from lab2.msg import Movement
import math

DEBUG = True
xDepressed = False
bDepressed = False
#targetRate= float(rospy.get_param('~targetRate','10'))
targetRate=0
first = False
previousError=0

def printD(string):
	global DEBUG
	if DEBUG:
		print string

def listener():
	rospy.init_node('AngularVelocityPID')
	rospy.Subscriber('/hovercraft/Gyro',Gyro,gyroCallback)
	rospy.Subscriber('/angularPositionOut',Movement,positionCallback)
	rospy.spin()

def positionCallback(move):
	global targetRate
	targetRate = move.theta

def gyroCallback(gyro):
	global targetRate
	global previousError
	global DEBUG
	global first
	pub = rospy.Publisher('/thrusterMapping',Movement)
	move = Movement()
	if first:
		targetRate = gyro.rate
		first = False
	P = float(rospy.get_param('~P', '.01'))
	D = float(rospy.get_param('~D', '.01'))
	printD(P)
	r = P*(targetRate - gyro.rate)+D*((targetRate - gyro.rate)-previousError)
	#TODO Derivative part
	previousError = targetRate - gyro.rate
	if DEBUG:
		print "TargetRate:",targetRate,"\tGyro Rate:",gyro.rate,"\tr:",r,"\tRate Difference:",previousError
	move.theta = r #if math.fabs(targetAngle - gyro.angle) > 2 else 0
	if math.fabs(targetRate - gyro.rate) < 3:
		move.theta = 0
	move.x=0
	move.y=0
	pub.publish(move)

if __name__ == '__main__':
	listener() 
