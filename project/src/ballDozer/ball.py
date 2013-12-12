#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('lab3')
roslib.load_manifest('hoverboard')
from hoverboard.msg import PWMRaw
from lab3.msg import Movement, Range, DetectionSwitcher
from ballDetector.msg import ballVisible, ballLocation
from geometry_msgs.msg import Vector3

numPublishTimes = 15
class Ball():
    def __init__(self):
        self.debug = float(rospy.get_param('~debug', '0'))
        self.state = 0
        self.ballLocation = ballLocation()

        self.ballVisible  = False
        self.ballCaptured = False

        self.previousColor= None
        self.currentColor = 'yellow'
        self.targetColor  = None

        rospy.Subscriber('/ballLocation',ballLocation,self.getBallLocation)
        rospy.Subscriber('ballVisible',ballVisible, self.isBallVisible)
        rospy.Subscriber('/rangeInfo', Range, self.isBallCaptured)
        rospy.Timer(rospy.Duration(.1), self.stateMachine)
        print("Ball Initialized")
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
        if (rDistCM < 11 and rDistCM > 0): # (lDistCM < 9 and lDistCM > 0) or
            self.ballCaptured = True
        else:
            self.ballCaptured = False

    def stateMachine(self, timerStuff):
        publisher = rospy.Publisher('ballCollectionMovement', Movement)
        moveMessage = None
        rest = PWMRaw(channel=5, pwm=0)
        servoPublisher = rospy.Publisher('/hoverboard/PWMRaw/', PWMRaw)
        detectionPublisher = rospy.Publisher('/switcher/visArbitrator', DetectionSwitcher)
        one  = PWMRaw(channel=5, pwm=2)
        detectionSwitch = DetectionSwitcher()
        detectionSwitch.state = 2
        detectionPublisher.publish(detectionSwitch)
        if (self.state == 0): #Find a ball of any color
            #detectionSwitch.state = 2
            #detectionPublisher.publish(detectionSwitch)
            for i in xrange(0,numPublishTimes):
                servoPublisher.publish(one)
            moveMessage = self.detectBall()
        elif (self.state == 1): #Move towards the ball
            for i in xrange(0,numPublishTimes):
                servoPublisher.publish(rest)
            #detectionPublisher.publish(detectionSwitch)
            moveMessage = self.moveToBall()
        elif (self.state == 2): #Collect the ball in the mechanism
            #servoPublisher.publish(rest)
            #detectionPublisher.publish(detectionSwitch)
            moveMessage = self.collectBall()
        else: #Reset to initial state
            self.publishColorToBallPy()
            moveMessage = self.stop()
            self.ballVisible  = False
            self.ballCaptured = False
            self.currentColor = 'yellow'
            self.targetColor  = None
            self.state = 0
            donePublisher = rospy.Publisher('currentBallState',  DetectionSwitcher) #Reuse of existing message type
            dState = DetectionSwitcher()
            dState.state = 0
            donePublisher.publish(dState)

        publisher.publish(moveMessage)

    def publishColorToBallPy(self):
        #Publish the collect ball color
        color = DetectionSwitcher()
        if self.currentColor == 'orange':
            color.state = 1
        elif self.currentColor == 'purple':
            color.state = 2
        elif self.currentColor == 'yellow':
            color.state = 3
        elif self.currentColor == 'green':
            color.state = 4
        elif self.currentColor == 'blue':
            color.state = 5
        publisher = rospy.Publisher('foundBallColor', DetectionSwitcher)
        publisher.publish(color)



    #This finds the ball.  We are going to rotate (maybe 30 degrees, according to field of vision of camera), then cycle through colors to see if it detects one.
    def detectBall(self):
        if self.hasFoundColor():
            #TODO: Need to publish self.targetColor to ballCleaner here
            self.state+=1 # NextStep: moveToBall
            self.debugPrint('Moving to Ball')
            return self.stop()
        else:
            return self.rotate(30)


    #Colors Available: red, yellow, blue, purple, green, orange
    #This starts from the current color and cycles through all the colors and checks to see if a ball is found,
    def hasFoundColor(self):
        numColors = 5
        for i in xrange(0, numColors):
            self.cycleColor()
            if self.ballVisible:
                self.targetColor = self.currentColor
                string = ('Detected a %s ball'%(self.currentColor))
                self.debugPrint(string)
                return True
            else:
                string = ('%s ball not found, checking next color.'%(self.currentColor.capitalize()))
                self.debugPrint(string)
        return False

    #Changes and publishes ball color to detect
    def cycleColor(self):
        lowPublisher = rospy.Publisher('thresh/low', Vector3)
        highPublisher= rospy.Publisher('thresh/high',Vector3)
        #if self.currentColor == 'red':
        #    self.currentColor = 'yellow'
        #    return detectYellowBall()
        if self.currentColor == 'yellow':
            self.previousColor='yellow'
            self.currentColor = 'blue'
            hsvLow, hsvHigh = self.detectBlueBall()
        elif self.currentColor == 'blue':
            self.previousColor='blue'
            self.currentColor = 'purple'
            hsvLow, hsvHigh = self.detectPurpleBall()
        elif self.currentColor == 'purple':
            self.previousColor='purple'
            self.currentColor = 'green'
            hsvLow, hsvHigh =  self.detectGreenBall()
        elif self.currentColor == 'green':
            self.previousColor='green'
            self.currentColor = 'orange'
            hsvLow, hsvHigh =  self.detectOrangeBall()
        elif self.currentColor == 'orange':
            self.previousColor= 'orange'
            self.currentColor = 'yellow'
            hsvLow, hsvHigh = self.detectYellowBall()
        self.debugPrint("Checking Color " + self.currentColor)
        for i in xrange(0,25): #Might need to lower number of iterations
            lowPublisher.publish(hsvLow)
            highPublisher.publish(hsvHigh)
        rospy.sleep(2.)

    #Moves towards the ball while centering it
    def moveToBall(self):
        #Need to change the state at some point
        if not self.ballCaptured:
            return self.move(y=.7,theta=(-float(self.ballLocation.x)/2.0))
        else:
            self.state+=1
            self.debugPrint('Loading Ball')
            return self.stop()

    def collectBall(self):
        servoPub = rospy.Publisher('/hoverboard/PWMRaw',PWMRaw)
        rest = PWMRaw(channel=5, pwm=0)
        one  = PWMRaw(channel=5, pwm=2)
        two  = PWMRaw(channel=5, pwm=7)
        #Might have to publish these more than once
        for i in xrange(0,numPublishTimes):
            servoPub.publish(rest)
        rospy.sleep(2)
        for i in xrange(0,numPublishTimes):
            servoPub.publish(one)
        rospy.sleep(2)
        for i in xrange(0,numPublishTimes):
            servoPub.publish(two)
        rospy.sleep(2)
        for i in xrange(0,numPublishTimes):
            servoPub.publish(rest)
        self.state+=1
        return self.stop()

    #Values for various colored balls
    #NOTE: self parameter removed intentionally, should work
    #NOTE: I hope keyword instantiation works
    def detectRedBall(self):
        low = Vector3(x=109,y=81,z=115)
        high= Vector3(x=122,y=186,z=255)
        return low,high
    def detectYellowBall(self):
        low = Vector3(x=73,y=69,z=136)
        high= Vector3(x=97,y=161,z=255)
        return low,high
    def detectBlueBall(self):
        low = Vector3(x=0,y=154,z=101)
        high= Vector3(x=17,y=255,z=249)
        return low,high
    def detectPurpleBall(self):
        low = Vector3(x=129,y=60,z=43)
        high= Vector3(x=167,y=126,z=196)
        return low,high
    def detectGreenBall(self):
        low = Vector3(x=28,y=111,z=73)
        high= Vector3(x=46,y=231,z=205)
        return low,high
    def detectOrangeBall(self):
        low = Vector3(x=98,y=64,z=167)
        high= Vector3(x=125,y=175,z=255)
        return low,high

    #Spins the ball
    def spin(self):
        return self.move(theta = -15)

    #Stopping the hovercraft
    def stop(self):
        return self.move()

    #Rotating the hovercraft (-theta is clockwise)
    def rotate(self, theta1):
        return self.move(theta=theta1)

    #Translating the hovercraft (+y is forward)
    def translate(self, x1=0, y1=0):
        return self.move(x=x1, y=y1)

    def move(self, x=0, y=0, theta=0):
        move = Movement()
        move.x = x
        move.y = y
        move.theta = theta
        move.modType = 'Add'
        return move

    def debugPrint(self,string):
        if self.debug == 1:
            print string

if __name__=='__main__':
    rospy.init_node('BallMover')
    try:
        ne = Ball()
        rospy.spin()
    except rospy.ROSInterruptException: pass
