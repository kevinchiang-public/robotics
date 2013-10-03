#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('hovercraft')
from hovercraft.msg import Thruster
from sensor_msgs.msg import Joy
import math

'''
Notes:

Start button logic might need some comments

Python Ternary Operator
variable = (True) if (Condition) else (False)

C-style Ternary operator
variable = (Condition)?(True):(False)
'''

#Set DEBUG to True if you want to display the variables in console
#Not logged
DEBUG=True

#Use printD to enable DEBUG toggling functionality
def printD(string):
        global DEBUG
        if DEBUG:
                print string

#Configuration
#This is all that you should be changing.
class config():
        lift = .35 #Absolute value, there is no further modification to lift
        t1mod= 1
        t2mod= 1
        t3mod= 1
        t4mod= 1
        t5mod= 1
        tAmod= .7 # Multiplies all thrusters (except lift) further by this number

class XboxTeleop():
        thrusterPub = rospy.Publisher('/joyOut',Thruster)
        liftOn = False
        startButtonDepressed = False

#Create the XboxTeleop object
xbox = XboxTeleop()

#This gets called when the node is launched
def listener():
        printD("Xbox Teleop spinning")
        rospy.init_node('XboxTeleop')
        rospy.Subscriber("/joy/",Joy,callback)
        rospy.spin()

#This gets called when /joy topic broadcasts a message
def callback(joyData):
        #I'm justifying using globals by saying that I'm not sure of the behavior
        #of rospy.Subscriber when passing additional arguments to callback
        global DEBUG
        global xbox
        
        const = config()

        #It would be unwise to change these
        start=joyData.buttons[7]
        yAxisR=joyData.axes[4] # 'R' for right
        xAxisR=joyData.axes[3]
        rotateAxis=joyData.axes[0]

        
        liftPower = const.lift
        thrustModifier = const.tAmod #Max thruster power of .5
        thrust = Thruster()
        rotatePower = math.fabs(rotateAxis)
        
        #Hypotenuse of x and y axis on right joystick
        #Dampened to 1 if necessary
        translatePower = math.sqrt(xAxisR*xAxisR+yAxisR*yAxisR)
        translatePower = 1 if translatePower > 1 else translatePower
        
        #Get the arctangent of yAxis/xAxis to get the angle in radians.
        #Convert it to degrees and make it so that it goes from 0-360 starting
        #at the positive x-axis
        angle = round(math.atan2(yAxisR,xAxisR)*(180.0/3.141593)+180,4)
        
        #Angle normalization for when we calculate the Thruster power
        t1angle = angle+360. if angle < 330. else angle
        t2angle = angle+360. if angle < 210. else angle
        t3angle = angle
        
        t1crit = 450.0
        t2crit = 330.0
        t3crit = 210.0
        increment = 1.0/120.0

        if start == 1 and not xbox.startButtonDepressed:
                xbox.startButtonDepressed = True
                printD("Start Button Pressed")
                thrust.thruster1 = 0
                thrust.thruster2 = 0
                thrust.thruster3 = 0
                thrust.thruster4 = 0
                thrust.thruster5 = 0
                
                if xbox.liftOn:
                        printD("Turning Thruster Off")
                        xbox.liftOn = False
                        thrust.lift = 0
                else:
                        printD("Turning Thruster On")
                        thrust.lift = liftPower
                        xbox.liftOn = True

                xbox.thrusterPub.publish(thrust)

        elif start == 0 and xbox.startButtonDepressed:
                xbox.startButtonDepressed = False



#Entry point for ROS
if __name__ == '__main__':
        listener()
