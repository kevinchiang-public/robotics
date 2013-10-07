#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('lab2')
from sensor_msgs.msg import Joy
from lab2.msg import MovementRaw, Movement
import math

def listener():
	rospy.init_node('JoystickAngleIntegrator')
	rospy.Subscriber('/joyOut', MovementRaw, callback)
	rospy.spin()

def callback(move):
	publisher = rospy.Publisher('/angleIntegratorOut',Movement)
	xAxisL = move.xL
	yAxisL = move.yL
	xAxisR = move.xR
	xAxisR = move.yR

	
	#Get the arctangent of yAxis/xAxis to get the angle in radians.
        #Convert it to degrees and make it so that it goes from 0-360 starting
        #at the positive x-axis
	rotationalAngle = round(math.atan2(yAxisL,xAxisL)*(180.0/3.141593)+180,4)
	
	
if __name__ == '__main__':
        listener() 
