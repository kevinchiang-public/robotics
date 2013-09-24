#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('hovercraft')
from hovercraft.msg import Thruster
from sensor_msgs.msg import Joy
import math

'''
Notes:

Start button logic might need some comments

Python Ternary Operator
variable = (True) if (Condition) else (False)

C-style Ternary operator
variable = (Condition)?(True):(False)
'''

#Set DEBUG to True if you want to display the variables in console
#Not logged
DEBUG=True

#Use printD to enable DEBUG toggling functionality
def printD(string):
	global DEBUG
	if DEBUG:
		print string

class XboxTeleop():
	thrusterPub = rospy.Publisher('hovercraft/Thruster',Thruster)
	liftOn = False
	startButtonDepressed = False

#Create the XboxTeleop object
xbox = XboxTeleop()

#This gets called when the node is launched
def listener():
	printD("Xbox Teleop spinning")
	rospy.init_node('XboxTeleop')
	rospy.Subscriber("/joy/",Joy,callback)
	rospy.spin()

#This gets called when /joy topic broadcasts a message
def callback(joyData):
	#I'm justifying using globals by saying that I'm not sure of the behavior
	#of rospy.Subscriber when passing additional arguments to callback
	global DEBUG
	global xbox
	
	#It would be unwise to change these
	start=joyData.buttons[7]
	yAxisR=joyData.axes[4] # 'R' for right
	xAxisR=joyData.axes[3]
	rotateAxis=joyData.axes[0]

	
	liftPower = .4
	thrustModifier = .5 #Max thruster power of .5
	thrust = Thruster()
	rotatePower = math.fabs(rotateAxis)
	
	#Hypotenuse of x and y axis on right joystick
	#Dampened to 1 if necessary
	translatePower = math.sqrt(xAxisR*xAxisR+yAxisR*yAxisR)
	translatePower = 1 if translatePower > 1 else translatePower
	
	#Get the arctangent of yAxis/xAxis to get the angle in radians.
	#Convert it to degrees and make it so that it goes from 0-360 starting
	#at the positive x-axis
	angle = round(math.atan2(yAxisR,xAxisR)*(180.0/3.141593)+180,4)
	
	#Angle normalization for when we calculate the Thruster power
	t1angle = angle+360. if angle < 330. else angle
	t2angle = angle+360. if angle < 210. else angle
	t3angle = angle
	
	t1crit = 450.0
	t2crit = 330.0
	t3crit = 210.0
	increment = 1.0/120.0

	if start == 1 and not xbox.startButtonDepressed:
		xbox.startButtonDepressed = True
		printD("Start Button Pressed")
		thrust.thruster1 = 0
		thrust.thruster2 = 0
		thrust.thruster3 = 0
		thrust.thruster4 = 0
		thrust.thruster5 = 0
		
		if xbox.liftOn:
			printD("Turning Thruster Off")
			xbox.liftOn = False
			thrust.lift = 0
		else:
			printD("Turning Thruster On")
			thrust.lift = liftPower
			xbox.liftOn = True

		xbox.thrusterPub.publish(thrust)

	elif start == 0 and xbox.startButtonDepressed:
		xbox.startButtonDepressed = False

	if xbox.liftOn:
		thrust.lift = liftPower
		thrust.thruster1 = 0
		thrust.thruster2 = 0
		thrust.thruster3 = 0
		thrust.thruster4 = 0
		thrust.thruster5 = 0

		#Rotational Controls
		if rotateAxis < -.1:
			thrust.thruster4 = 0
			thrust.thruster5 = rotatePower * thrustModifier
		elif rotateAxis > .1:
			thrust.thruster4 = rotatePower * thrustModifier
			thrust.thruster5 =0
	
		#Translation Controls

		'''
		The theory behind this:
		Orientation:  Right is 0 degrees.  Increasing in clockwise direction. 360 is 0
		There are three critical points:  90, 210, and 330 degrees.  These are the points where only
		one thruster is needed.

		So the thruster1 range should increase from -30 degrees to 90 degrees, and decrease from
		90 to 210 degrees. I'm assuming it should be continuous and equally distributed.
		If the power for thruster1 is 1 at 90 degrees, then it should be 1-(1/120) at 89 degrees,
		or 91 degrees. Etc...

		The angles are normalized so I don't have to worry about negative numbers.
		'''

		#Thruster1
		if t1angle < t1crit:
			thrust.thruster1 = 1.0 - ((t1crit - t1angle)*(increment))
		elif t1angle == t1crit:
			thrust.thruster1 = 1.0
		elif t1angle > t1crit:
			thrust.thruster1 = 1.0 - ((t1angle - t1crit)*(increment))

		#Thruster2
		if t2angle < t2crit:
			thrust.thruster2 = 1.0 - ((t2crit - t2angle)*(increment))
		elif t2angle == t2crit:
			thrust.thruster2 = 1.0
		elif t2angle > t2crit:
			thrust.thruster2 = 1.0 - ((t2angle - t2crit)*(increment))
		
		#Thruster3
		if t3angle < t3crit:
			thrust.thruster3 = 1.0 - ((t3crit - t3angle)*(increment))
		elif t3angle == t3crit:
			thrust.thruster3 = 1.0
		elif t3angle > t3crit:
			thrust.thruster3 = 1.0 - ((t3angle - t3crit)*(increment))

		#Power Dampening and Correction
		thrust.thruster1 = 0 if thrust.thruster1 < 0 else thrust.thruster1
		thrust.thruster2 = 0 if thrust.thruster2 < 0 else thrust.thruster1
		thrust.thruster3 = 0 if thrust.thruster3 < 0 else thrust.thruster1

		thrust.thruster1 = thrust.thruster1 * thrustModifier * translatePower
		thrust.thruster2 = thrust.thruster2 * thrustModifier * translatePower
		thrust.thruster3 = thrust.thruster3 * thrustModifier * translatePower

		#Debug
		if DEBUG and xbox.liftOn:
			print "xAxisR:",xAxisR,"\tyAxisR",yAxisR,"\tRotate Axis:",rotateAxis
			print "LiftStatus:",xbox.liftOn,"\tStartButtonDepressed",xbox.startButtonDepressed
			print "Angle",angle,"\tt1angle:",t1angle,"\tt2angle:",t2angle,"\tt3angle:",t3angle
			print "Thruster1:",thrust.thruster1,"\tThruster2:",thrust.thruster2
			print "Thruster3:",thrust.thruster3,"\tThruster4:",thrust.thruster4
			print "Thruster5:",thrust.thruster5,"\tLift:",thrust.lift,"\n"

		xbox.thrusterPub.publish(thrust)

#Entry point for ROS
if __name__ == '__main__':
	listener()
