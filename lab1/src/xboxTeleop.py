#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
import roslib
roslib.load_manifest('hovercraft')
from hovercraft.msg import Thruster

class XboxTeleop():
	pub = rospy.Publisher('/hovercraft/Thruster',joy)
	thrusterOn= False
	startButtonDepressed = False

def listener():
	rospy.init_node('xboxTeleop', anonymous=True)
	rospy.Subscriber('/joy',Joy, joyCallback)
	rospy.spin()
	
def joyCallback(data):
	joy=data
	
def action(joy):
	global xbox
	wasPressed = False
	lift = 0
	thruster1 = 0
	thruster2 = 0
	thruster3 = 0
	thruster4 = 0
	thruster5 = 0

	print "joyCallback executed"
	if joy.buttons[7] == 1 and not xbox.startButtonDepressed:
		wasPressed = True
		xbox.startButtonDepressed = True
		print "Start Button Pressed"
	
		if xbox.ThrusterOn:
			print "Turning Thruster Off"
			xbox.thrusterOn = False
			lift = 0 
			thruster1 = 0
			thruster2 = 0
			thruster3 = 0
			thruster4 = 0
			thruster5 = 0
		else:
			print "Turning Thruster On"
			lift = .4
			xbox.thrusterOn = True
			
	else:
		xbox.startButtonDepressed= False
		
	if wasPressed:
		thrust = Thruster()
		thrust.lift = lift;
		thrust.thruster1 = thruster1;
		thrust.thruster2 = thruster2;
		thrust.thruster3 = thruster3;
		thrust.thruster4 = thruster4;
		thrust.thruster5 = thruster5;
		xbox.pub.Publish(thrust)
		
if __name__ == '__main__':
	xbox = XboxTeleop
	listener()
