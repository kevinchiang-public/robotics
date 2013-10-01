#!/usr/bin/env python

import rospy
import roslib
roslib.load_manifest('hovercraft')
from hovercraft.msg import Gyro
from sensor_msgs.msg import Joy

class angularPositionPID():
	def __init__(self):
		self.targetAngle=None
		rospy.Subscriber('/hovercraft/Gyro',self.gyroCallback())
		rospy.Subscriber('/joy',joyCallback())
		self.xDepressed = False
		self.bDepressed = False

	def joyCallback(joy):
		#TODO get the buttons
		#self.aButton = joy.buttons[0]
		bButton = joy.buttons[1]
		xButton = joy.buttons[2]
		#self.yButton = joy.buttons[3]
		if bButton == 1 and not self.bDepressed:
			self.bDepressed = True
			print("B button Pushed")
			self.targetAngle += 90
		elif xButton == 1 and not self.xDepressed:
			self.xDepressed = True
			print ("X button Pushed")
			self.targetAngle += -90
		
		if bButton == 0 and self.bDepressed:
			self.bDepressed = False
		if xButton == 0 and self.xDepressed:
			self.xDepressed = False

		

	def gyroCallback(gyro):
		#Main gyro PID logic
		if self.targetAngle is None:
			self.targetAngle = gyro.angle
		
		


if __name__ == '__main__':
	rospy.init_node('AngularPositionPID')
	try:
		angularPositionPID = angularPositionPID()
	except rospy.ROSInterruptException: pass
