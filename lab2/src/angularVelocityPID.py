#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('lab2')
roslib.load_manifest('hovercraft')
from hovercraft.msg import Gyro
from sensor_msgs.msg import Joy
from lab2.msg import Movement

previousRate = 0

def listener():
        rospy.init_node('AngularVelocityPID')
        rospy.Subscriber('/hovercraft/Gyro',Gyro,gyroCallback)
        rospy.Subscriber('/joy',Joy,joyCallback)
        rospy.spin()

def gyroCallback(gyro):
	global previousRate

if __name__ == '__main__':
        listener()
                       
