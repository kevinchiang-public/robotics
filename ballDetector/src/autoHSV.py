#!/usr/bin/env python
import rospy
import roslib
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
class AutoHSV():
	def __init__(self):
		self.threshHighPub = rospy.Publisher('thresh/high',Vector3)
		self.threshLowPub  = rospy.Publisher('thresh/low' ,Vector3)
		#publish(x=lh,y=ls,z=lv)
		rospy.Subscriber('/balldebug/img3',Image,self.hsvCallback)
		
	def hsvCallback(self,image):
		H = []
		S = []
		V = []
		#Seperate the image into HSV values
		for x in xrange(0, len(image.data), 3):
			if image.data[x] < 64 or image.data[x] > 108:
				H.append(ord(image.data[x]))
		for x in xrange(1, len(image.data), 3):
			if image.data[x] < 10 or image.data[x] > 70:
				S.append(ord(image.data[x]))
		for x in xrange(2, len(image.data), 3):
			if image.data[x] < 105 or image.data[x] > 154:
				V.append(ord(image.data[x]))
		#print H
		
		lh = min(H)
		ls = min(S)
		lv = min(V)
		hh = max(H)
		hs = max(S)
		hv = max(V)
		#print ("LH:%d\tLS:%d\tLV:%d\tHH:%d\tHS:%d\tHV:%d"%(lh, ls, lv, hh, hs, hv))
		self.threshHighPub.publish(x=hh,y=hs,z=hv)
		self.threshLowPub.publish(x=lh,y=ls,z=lv)



if __name__ == '__main__':
	rospy.init_node('autoHSV')
	try:
		ne = AutoHSV()
		rospy.spin()
	except rospy.ROSInterruptException: pass
