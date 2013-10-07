#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('hovercraft')
roslib.load_manifest('lab2')
from sensor_msgs.msg import Joy
from lab2.msg import MovementRaw
from lab2.msg import Switcher
import math


#Set DEBUG to True if you want to display the variables in console
#Not logged
DEBUG=True
xbox = None
switchState = 0 # Hovercraft responds to joystick movement controls on startup

#Use printD to enable DEBUG toggling functionality
def printD(string):
        global DEBUG
        if DEBUG:
                print string

class XboxTeleop():
        angleIntegratorPub = rospy.Publisher('/joyOut',MovementRaw)
	arbitratorPub=rospy.Publisher('/joyArbitrator',Switcher)
	movement = MovementRaw()
	switcher = Switcher()
        liftOn = False
        startButtonDepressed = False
	leftButtonDepressed  = False
	rightButtonDepressed = False

def callback(joyData):
	global switchState
	#Switcher reminder:
	JOYSTICK = 0
	TRIANGLE = 1
	REACTIVE = 2

	global xbox# = XboxTeleop()
	start = joyData.buttons[7]
	leftButton = joyData.buttons[4]
	rightButton= joyData.buttons[5]

	#TODO Double check the left axes x and y
	xAxisL = joyData.axes[0]
	yAxisL = joyData.axes[1]
	yAxisR = joyData.axes[4]
	xAxisR = joyData.axes[3]
	
	#Start Button Logic
	if start == 1 and not xbox.startButtonDepressed:
		xbox.startButtonDepressed = True
		printD("Start Button Pressed")

		if xbox.liftOn:
			printD("Turning Thruster Off")
			xbox.liftOn = False
			xbox.switcher.lift = False
		else:
			printD("Turning Thruster On")
			xbox.liftOn = True
			xbox.switcher.lift = True
	
	elif start == 0 and xbox.startButtonDepressed:
		xbox.startButtonDepressed = False

	#Left Button Logic
	if leftButton == 1 and not xbox.leftButtonDepressed:
		xbox.leftButtonDepressed = True
		printD("Left Button Pressed")

		#Cycle left (JOYSTICK, TRIANGLE, REACTIVE)
		if switchState == JOYSTICK:
			switchState = REACTIVE
			printD("Reactive Control Activated\n")
			xbox.switcher.state = switchState
		elif switchState == REACTIVE:
			switchState = TRIANGLE
			printD("Triangle Activated\n")
			xbox.switcher.state = switchState
		elif switchState == TRIANGLE:
			switchState = JOYSTICK
			printD("Joystick Control Activated\n")
			xbox.switcher.state = switchState

	elif leftButton == 0 and xbox.leftButtonDepressed:
		xbox.leftButtonDepressed = False

	#Right Button Logic
	#Right button may have priority, but is untested
	if rightButton == 1 and not xbox.rightButtonDepressed:
		xbox.rightButtonDepressed = True
		printD("Right Button Pressed")
		
		#Cycle Right (JOYSTICK, TRIANGLE, REACTIVE)
		if switchState == JOYSTICK:
			switchState = TRIANGLE
			printD("Triangle Activated\n")
			xbox.switcher.state = switchState
		elif switchState == TRIANGLE:
			switchState = REACTIVE
			printD("Reactive Control Activated\n")
			xbox.switcher.state = switchState
		elif switchState == REACTIVE:
			switchState = JOYSTICK
			printD("Joystick Control Activated\n")
			xbox.switcher.state = switchState
	elif rightButton == 0 and xbox.rightButtonDepressed:
		xbox.rightButtonDepressed = True


	#Translational Control Passthrough
	xbox.movement.xR = xAxisR
	xbox.movement.yR = yAxisR
	xbox.movement.xL = xAxisL
	xbox.movement.yL = yAxisL

	

	#Publish to AngleIntegrator
	xbox.angleIntegratorPub.publish(xbox.movement)

	#Publish to Arbitrator
	xbox.arbitratorPub.publish(xbox.switcher)
		
def listener():
	rospy.init_node('joystick')
	printD("Joystick Spinning")
	global xbox
	xbox = XboxTeleop()
	rospy.Subscriber('/joy',Joy, callback)
	rospy.spin()

if __name__ == '__main__':
	listener()


