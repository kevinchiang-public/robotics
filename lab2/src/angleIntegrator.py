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


        #Get the arctangent of xAxis/yAxis to get the angle in radians.
        #Convert it to degrees and make it so that it goes from 0-360 starting
        #at the positive y axis (to match with the front of the hovercraft)
        rotationalAngle = round(math.atan2(xAxisL,yAxisL)*(180.0/3.141593),4)
        if (rotationalAngle > 0):
                rotationalAngle = rotationalAngle - 360
        rotationalAngle = math.fabs(rotationalAngle)
        print "xAxisL: " + str(xAxisL) + "\tyAxisL: " + str(yAxisL) + "\tRotational Angle: " + str(rotationalAngle)

if __name__ == '__main__':
        listener()
