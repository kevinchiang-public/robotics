#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('lab3')
from lab3.msg import Movement, Range
from ballDetector.msg import ballVisible, ballLocation
from geometry_msgs.msg import Vector3
class Ball():
    def __init__(self):
        self.debug = float(rospy.get_param('~debug', '0'))

        self.state = 0

        self.ballLocation = ballLocation()

        self.ballVisible  = False
        self.ballCaptured = False

        self.currentColor = 'yellow'
        self.targetColor  = None

        rospy.Subscriber('/ballLocation',ballLocation,self.getBallLocation)
        rospy.Subscriber('ballVisible',ballVisible, self.isBallVisible)
        rospy.Subscriber('/rangeInfo', Range, self.isBallCaptured)

    #Gets the ball Location Data
    def getBallLocation(self,ballLocation):
        self.ballLocation = ballLocation

    #Checks to see if the ball is in view
    def isBallVisible(self,ballVis):
        if(ballVis.visible == 1):
            self.ballVisible = True
        else:
            self.ballVisible = False

    def isBallCaptured(self,rangeInfo):
        lDistCM = rangeInfo.leftDistanceCM
        rDistCM = rangeInfo.rightDistanceCM
        if (lDistCM < 9 and lDistCM > 0) or (rDistCM < 15 and rDistCM > 0):
            self.ballCaptured = True
        else:
            self.ballCaptured = False

    def stateMachine(self):
        pass

    #This finds the ball.  We are going to rotate (maybe 30 degrees, according to field of vision of camera), then cycle through colors to see if it detects one.
    def detectBall(self):
        if hasFoundColor:
            self.state+=1 # NextStep: moveToBall
        else:
            self.rotate(30)


    #Colors Available: red, yellow, blue, purple, green, orange
    #This starts from the current color and cycles through all the colors and checks to see if a ball is found,
    def hasFoundColor(self):
        numColors = 5
        for i in xrange(0, numColors):
            self.cycleColor()
            if self.ballCaptured:
                self.targetColor = self.currentColor
                string = ('Detected a %s ball'%(self.targetColor))
                debugPrint(string)
                return True 
            else:
                string = ('%s ball not found, checking next color.'%(self.targetColor.capitalize))
                debugPrint(string)
        return False

    #Changes and publishes ball color to detect
    def cycleColor(self):
        lowPublisher = rospy.Publisher('thresh/low', Vector3)
        highPublisher= rospy.Publisher('thresh/high',Vector)
        #if self.currentColor == 'red':
        #    self.currentColor = 'yellow'
        #    return detectYellowBall()
        if self.currentColor == 'yellow':
            self.currentColor = 'blue'
            hsvLow, hsvHigh = detectBlueBall()
        elif self.currentColor == 'blue':
            self.currentColor = 'purple'
            hsvLow, hsvHigh = detectPurpleBall()
        elif self.currentColor == 'purple':
            self.currentColor = 'green'
            hsvLow, hsvHigh =  detectGreenBall()
        elif self.currentColor == 'green':
            self.currentColor = 'orange'
            hsvLow, hsvHigh =  detectOrangeBall()
        elif self.currentColor == 'orange':
            self.currentColor = 'yellow'
            hsvLow, hsvHigh = detectYellowBall()
        for i in xrange(0,25):
            lowPublisher.publish(hsvLow)
            highPublisher.publish(hsvHigh)

    def moveToBall(self):
        pass

    def collectBall(self):
        pass

    def loadBall(self):
        pass
    
    #Values for various colored balls
    #NOTE: self parameter removed intentionally, should work
    #NOTE: I hope keyword instantiation works
    def detectRedBall():
        low = Vector3(x=109,y=81,z=115)
        high= Vector3(x=122,y=186,z=255)
        return low,high
    def detectYellowBall():
        low = Vector3(x=73,y=69,z=136)
        high= Vector3(x=97,y=161,z=255)
        return low,high
    def detectBlueBall():
        low = Vector3(x=0,y=154,z=101)
        high= Vector3(x=17,y=255,z=249)
        return low,high
    def detectPurpleBall():
        low = Vector3(x=129,y=60,z=43)
        high= Vector3(x=167,y=126,z=196)
        return low,high
    def detectGreenBall():        
        low = Vector3(x=28,y=111,z=73)
        high= Vector3(x=46,y=231,z=205)
        return low,high
    def detectOrangeBall():
        low = Vector3(x=98,y=64,z=167)
        high= Vector3(x=125,y=175,z=255)
        return low,high

    #Spins the ball
    def spin(self):
        move = Movement()
        move.modType = 'Add'
        move.theta = -15
        move.x = 0
        move.y = 0
        return move

    #Stopping the hovercraft
    def stop(self):
        haltMove = Movement()
        haltMove.theta = 0
        haltMove.x = 0
        haltMove.y = 0
        haltMove.modType = 'Add'
        return haltMove

    #Rotating the hovercraft (-theta is clockwise)
    def rotate(self, theta):
        rotateMove = Movement()
        rotateMove.theta = theta
        rotateMove.x = 0
        rotateMove.y = 0
        rotateMove.modType = 'Add'
        return rotateMove

    #Translating the hovercraft (-y is forward)
    def translate(self, x, y=0):
        translateMove = Movement()
        translateMove.theta = 0
        translateMove.x = x
        translateMove.y = y
        translateMove.modType = 'Add'
        return translateMove

    def debugPrint(self,string):
        if self.debug == 1:
            print string

if __name__=='__main__':
    rospy.init_node('BallMover')
    try:
        ne = Ball()
        rospy.spin()
    except rospy.ROSInterruptException: pass