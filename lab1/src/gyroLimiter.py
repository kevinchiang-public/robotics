#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('hovercraft')
from hovercraft.msg import Thruster,Gyro
import time

#Set DEBUG to True if you want to display the variables in console
#Not logged
DEBUG=True
gyroRate=0
gyroAngle=0
gyroTime=0
#Use printD to enable DEBUG toggling functionality
def printD(string):
        global DEBUG
        if DEBUG:
                print string

def listener():
	printD("Gyro Limiter spinning")
	rospy.init_node('GyroLimiter')
	rospy.Subscriber("/hovercraft/Gyro/", Gyro, gyroCallback)
	rospy.Subscriber("/thrusterProcess", Thruster, thrusterCallback)
	rospy.spin()

def gyroCallback(gyroData):
	global gyroRate
	global gyroAngle
	gyroRate = gyroData.rate
	gyroAngle= gyroData.angle

def thrusterCallback(thrusterData):

	ftime =  time.localtime(time.time())
	ttime = ftime[3]*3600+ftime[4]*60+ftime[5]

	global DEBUG
	global gyroRate
	global gyroAngle
	global gyroTime

	thrust = Thruster()
	thrust.lift = thrusterData.lift
	thrust.thruster1 = thrusterData.thruster1
	thrust.thruster2 = thrusterData.thruster2
	thrust.thruster3 = thrusterData.thruster3
	thrust.thruster4 = thrusterData.thruster4  #Counter Clockwise
	thrust.thruster5 = thrusterData.thruster5  #Clockwise

	if gyroRate < -355 :
		printD("Gyro Limiter Engaged -")
		gyroTime = ttime
		thrust.thruster5 = 0
	elif gyroRate > 620:
		printD("Gyro Limiter Engaged +")
		gyroTime = ttime
		thrust.thruster4 = 0

	if DEBUG:
		print "Gyro Rate:",gyroRate,"\tGyro Angle:",gyroAngle
		print "Gyro Time:",gyroTime,"\tTtime:",ttime,"\n"

	if (ttime - gyroTime) < 1:
		printD("Inhibited")
		thrust.thruster4=0
		thrust.thruster5=0

	pub = rospy.Publisher('/hovercraft/Thruster',Thruster)
	pub.publish(thrust)



if __name__ == '__main__':
	listener()
