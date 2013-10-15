#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('hovercraft')
roslib.load_manifest('lab2')
from sensor_msgs.msg import Joy
from lab2.msg import MovementRaw
from lab2.msg import Switcher
import math


class XboxTeleop():
    def __init__(self):
        self.debug = float(rospy.get_param('~debug', '0'))
        self.switchState = 0
        self.movement = MovementRaw()
        self.switcher = Switcher()

        #Initialize the lift to false so no thrusters fire on launch
        self.switcher.lift = False
        self.liftOn = False

        #Used to track button toggles
        self.startButtonDepressed = False
        self.leftButtonDepressed = False
        self.rightButtonDepressed = False

        rospy.Subscriber('/joy',Joy, self.joystickCallback)
        rospy.Timer(rospy.Duration(.1), self.timerCallback)
        self.debugPrint('Joystick Spinning')

    def joystickCallback(self, joyData):
        #Switcher reminder: (Variables to help with naming conventions)
        leftButton = joyData.buttons[4]
        rightButton= joyData.buttons[5]
        start = joyData.buttons[7]

        #Start Button Logic
        if start == 1 and not self.startButtonDepressed:
            self.startButtonDepressed = True
            self.debugPrint("Start Button Pressed")

            if self.liftOn:
                self.debugPrint("Turning Thruster Off")
                self.liftOn = False
                self.switcher.lift = False
            else:
                self.debugPrint("Turning Thruster On")
                self.liftOn = True
                self.switcher.lift = True

        elif start == 0 and self.startButtonDepressed:
            self.startButtonDepressed = False

        #Left Button Logic
        if leftButton == 1 and not self.leftButtonDepressed:
            self.leftButtonDepressed = True
            self.debugPrint("Left Button Pressed")

            #Cycle left (JOYSTICK, TRIANGLE, REACTIVE)
            self.switchState = 2 if self.switchState - 1 < 0 else self.switchState - 1
            self.debugPrint("State is: " +  str(self.switchState))
        elif leftButton == 0 and self.leftButtonDepressed:
            self.leftButtonDepressed = False

        #Right Button Logic
        if rightButton == 1 and not self.rightButtonDepressed:
            self.rightButtonDepressed = True
            self.debugPrint("Right Button Pressed")
            #Cycle right (Joystick ,triangle, reactive)
            self.switchState = 0 if self.switchState + 1 > 2 else self.switchState + 1
            self.debugPrint("State is: " +  str(self.switchState))
        elif rightButton == 0 and self.rightButtonDepressed:
            self.rightButtonDepressed = False

        #Translational Control Passthrough
        self.movement.xR = joyData.axes[3]
        self.movement.yR = joyData.axes[4]
        self.movement.xL = joyData.axes[0]
        self.movement.yL = joyData.axes[1]

        #Note: Flipped on purpose
        self.movement.xButton = joyData.buttons[1]
        self.movement.bButton = joyData.buttons[2]
        self.switcher.state=self.switchState

    def timerCallback(self, event):
        angleIntegratorPub = rospy.Publisher('/joyOut',MovementRaw)
        arbitratorPub = rospy.Publisher('/joyArbitrator',Switcher)
        angleIntegratorPub.publish(self.movement)
        arbitratorPub.publish(self.switcher)

    def debugPrint(self, stringToPrint):
        if (self.debug == 1):
            print(stringToPrint)

if __name__ == '__main__':
    rospy.init_node('XboxTeleop')
    try:
        ne = XboxTeleop()
        rospy.spin()
    except rospy.ROSInterruptException: pass
