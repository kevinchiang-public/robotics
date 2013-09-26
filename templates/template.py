#!/usr/bin/env python
import rospy
import roslib

#load manifest if you are using the messages in a custom package
#roslib.load_manifest('package_name_here')


DEBUG = True
SUBSCRIBE_TOPIC = ''
SUBSCRIBE_TYPE= ''
PUBLISH_TOPIC  = ''
PUBLISH_TYPE = ''

def listener():
	global SUBSCRIBE_TOPIC
	global SUBSCRIBE_TYPE
	rospy.init_node('')  #name here
	rospy.Subscriber(SUBSCRIBE_TOPIC,SUBSCRIBE_TYPE,callback)
	rospy.spin()

def callback():
	global PUBLISH_TOPIC
	global PUBLISH_TYPE
	pub = rospy.Publisher(PUBLISH_TOPIC,PUBLISH_TYPE)

if __name__ = '__main__':
	listener() 
