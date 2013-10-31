#!/usr/bin/env python
import rospy
import roslib
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
import numpy
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
			if ord(image.data[x+1]) < 10 or ord(image.data[x+1]) > 70:
				if ord(image.data[x+2]) < 105 or ord(image.data[x+2]) > 154:
					H.append(ord(image.data[x]))
					S.append(ord(image.data[x+1]))
					V.append(ord(image.data[x+2]))		
		'''	
		lh = min(H)
		ls = min(S)
		lv = min(V)
		hh = max(H)
		hs = max(S)
		hv = max(V)
		print ("LH:%d\tLS:%d\tLV:%d\tHH:%d\tHS:%d\tHV:%d"%(lh, ls, lv, hh, hs, hv))
		'''
		hAvg = numpy.mean(H)
		sAvg = numpy.mean(S)
		vAvg = numpy.mean(V)

		hStd = numpy.std(H)
		sStd = numpy.std(S)
		vStd = numpy.std(V)

		
		lh = hAvg - hStd
		ls = sAvg - sStd
		lv = vAvg - vStd
		hh = hAvg + hStd
		hs = sAvg + sStd
		hv = vAvg + vStd
		print ("LH:%f\tLS:%f\tLV:%f\tHH:%f\tHS:%f\tHV:%f"%(lh, ls, lv, hh, hs, hv))

		self.threshHighPub.publish(x=hh,y=hs,z=hv)
		self.threshLowPub.publish(x=lh,y=ls,z=lv)



if __name__ == '__main__':
	rospy.init_node('autoHSV')
	try:
		ne = AutoHSV()
		rospy.spin()
	except rospy.ROSInterruptException: pass
