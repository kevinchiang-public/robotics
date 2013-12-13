#!/usr/bin/env python
import rospy
import roslib
import math
roslib.load_manifest('lab3')
roslib.load_manifest('hoverboard')
from landmarkSelfSim.msg import landmarkVisible, landmarkLocation
from ballDetector.msg import ballVisible, ballLocation
from geometry_msgs.msg import Vector3
from lab3.msg import Movement, Range, DetectionSwitcher
from hoverboard.msg import PWMRaw

class BallCleaner():
    def __init__(self):
        self.debug = float(rospy.get_param('~debug', '0'))
        #Updates state by publishing different move messages
        rospy.Timer(rospy.Duration(.1), self.updateState)

        #Initialize vision arbitrator
        publisher = rospy.Publisher('/detectionSwitch', DetectionSwitcher)
        switcher = DetectionSwitcher()
        switcher.state = 2
        for i in xrange(0,25):
            switcher.header.stamp = rospy.Time.now()
            publisher.publish(switcher)

        self.colors = ['blue', 'purple', 'green', 'orange']
        #Maps colors to landmark numbers
        self.colorDict = dict()
        self.colorDict['blue'] = int(rospy.get_param('~blueLandmark', '2'))
        self.colorDict['green'] = int(rospy.get_param('~greenLandmark', '3'))
        self.colorDict['yellow'] = int(rospy.get_param('~yellowLandmark', '4'))
        self.colorDict['purple'] = int(rospy.get_param('~purpleLandmark', '5'))
        self.colorDict['orange'] = int(rospy.get_param('~orangeLandmark', '6'))

        #State vars for the substate machines for landmark and ball collection
        self.landmarkVars = dict([('inView', False),('visibleLandmark', -1),('landmarkX', 0), ('distance', -1), ('targetReached', False)])
        self.ballVars = dict([('visible', False),('location', ballLocation()),('captured', False), ('colorIndex', 0)])
        self.state = 0
        #Ball Subscriptions
        rospy.Subscriber('/ballLocation',ballLocation,self.getBallLocation)
        rospy.Subscriber('ballVisible',ballVisible, self.isBallVisible)
        rospy.Subscriber('/rangeInfo', Range, self.isBallCaptured)

        #Landmark subscrptions
        rospy.Subscriber('landmarkVisible',landmarkVisible, self.checkForLandmark)
        rospy.Subscriber('landmarkLocation', landmarkLocation, self.getDistanceToLandmark)
    
    def resetVariables(self):
        self.landmarkVars = dict([('inView', False),('visibleLandmark', -1),('landmarkX', 0), ('distance', -1), ('targetReached', False), ])
        self.ballVars = dict([('visible', False),('location', ballLocation()),('captured', False), ('colorIndex', 0)])

    def updateState(self,timerShit):
        #Controls the overall state of the node
        numPublishTimes = 10
        cameraPublisher = rospy.Publisher('/detectionSwitch', DetectionSwitcher)
        switcher = DetectionSwitcher()

        #Publishes for the lift
        servoPublisher = rospy.Publisher('/hoverboard/PWMRaw/', PWMRaw)
        liftDown = PWMRaw(channel=5, pwm=0)
        liftMiddle = PWMRaw(channel=5, pwm=2)

        movePublisher = rospy.Publisher('/ballCleanerOut', Movement)
        moveMessage = None

        if self.state == 0:
            servoPublisher.publish(liftMiddle)
            moveMessage = self.detectBall()
            switcher.state = 2
        elif self.state == 1:
            self.debugPrint('Moving to Ball')
            servoPublisher.publish(liftDown)
            moveMessage = self.moveToBall()
            switcher.state = 2
        elif self.state == 2:
            self.debugPrint('Loading Ball')
            moveMessage = self.collectBall()
            switcher.state = 2
        elif self.state == 3:
            string = 'Looking for landmark %d'%(self.colorDict[self.getCurrentColor()])
            self.debugPrint(string)
            moveMessage = self.lookForLandmark(self.colorDict[self.getCurrentColor()])
            switcher.state = 1
        elif self.state == 4:
            self.debugPrint('Moving Towards Landmark')
            moveMessage = self.moveTowardsLandmark()
            switcher.state = 1
        elif self.state == 5:
            self.debugPrint('Firing')
            moveMessage = self.launch()
            switcher.state = 1
        elif self.state == 6:
            self.debugPrint('Stop Firing')
            moveMessage = self.stopLaunching()
            switcher.state = 1
        elif self.state == 7: #Resets state machine back to default settings
            self.resetVariables()
            switcher.state = 2
            self.state = 0
            moveMessage = self.stop()
        else:
            print('Well that shouldn\'t have happened.  State is now: ' + str(self.state))

        switcher.header.stamp  =rospy.Time.now()
        cameraPublisher.publish(switcher)
        movePublisher.publish(moveMessage)

    def move(self, x=0, y=0, theta=0):
        #Returns some kind of movement (Factory method)
        move = Movement()
        move.x = x
        move.y = y
        move.theta = theta
        move.modType = 'Add'
        return move
    #Move helper functions
    def spin(self):
        return self.move(theta = -15)
    def stop(self):
        return self.move()
    def rotate(self, theta1):
        #-theta is clockwise
        return self.move(theta=theta1)
    def translate(self, x1=0, y1=0):
        #+y is forward
        return self.move(x=x1, y=y1)

    def debugPrint(self,string):
        if self.debug == 1:
            print string

    ###################
    #Ball Detection/Collection Functions#
    ###################
    def getBallLocation(self,ballLocation):
        #Gets the ball Location Data
        self.ballVars['location'] = ballLocation

    def isBallVisible(self,ballVis):
        #Checks to see if the ball is in view
        if(ballVis.visible == 1):
            self.ballVars['visible'] = True
        else:
            self.ballVars['visible'] = False

    def isBallCaptured(self,rangeInfo):
        #Uses the IR finder to see if the ball is in the capture mechanism
        lDistCM = rangeInfo.leftDistanceCM
        rDistCM = rangeInfo.rightDistanceCM
        if (rDistCM < 11 and rDistCM > 0): # (lDistCM < 9 and lDistCM > 0) or
            self.ballVars['captured'] = True
        else:
            self.ballVars['captured'] = False

    def detectBall(self):
        #Detects the ball by rotating 30 degrees and cycling through colors
        if self.hasFoundColor():
            self.state+=1 # NextStep: moveToBall
            return self.stop()
        else:
            return self.rotate(30)

    def hasFoundColor(self):
        #Cycles through colors looking for any visible balls
        for i in xrange(0, len(self.colors)):
            self.cycleColor()
            if self.ballVars['visible']:
                string = ('Detected a %s ball'%(self.getCurrentColor()))
                self.debugPrint(string)
                return True
            else:
                string = ('%s ball not found, checking next color.'%(self.getCurrentColor().capitalize()))
                self.debugPrint(string)
        return False

    def cycleColor(self):
        #Changes and publishes ball color to detect
        lowPublisher = rospy.Publisher('thresh/low', Vector3)
        highPublisher= rospy.Publisher('thresh/high',Vector3)

        #Updates the current color
        self.ballVars['colorIndex'] += 1
        if self.ballVars['colorIndex'] >= len(self.colors):
            self.ballVars['colorIndex'] = 0
        hsvLow, hsvHigh = self.getHSVVectors()
        self.debugPrint("Checking Color " + self.colors[self.ballVars['colorIndex']])
        for i in xrange(0,25): #Might need to lower number of iterations
            lowPublisher.publish(hsvLow)
            highPublisher.publish(hsvHigh)
        rospy.sleep(2)

    def getCurrentColor(self):
        return self.colors[self.ballVars['colorIndex']]

    def getHSVVectors(self):
        currentColor = self.getCurrentColor()
        if currentColor == 'red':
            return Vector3(x=109,y=81,z=115), Vector3(x=122,y=186,z=255)
        elif currentColor == 'yellow':
            return Vector3(x=73,y=69,z=136), Vector3(x=97,y=161,z=255)
        elif currentColor == 'blue':
            return Vector3(x=0,y=154,z=101), Vector3(x=17,y=255,z=249)
        elif currentColor == 'purple':
            return Vector3(x=129,y=60,z=43), Vector3(x=167,y=126,z=196)
        elif currentColor == 'green':
            return Vector3(x=28,y=111,z=73), Vector3(x=46,y=231,z=205)
        elif currentColor == 'orange':
            return Vector3(x=98,y=64,z=167), Vector3(x=125,y=175,z=255)
        else:
            self.debugPrint('Major Error: color ' + str(self.ballVars['colorIndex']) +' not found')

    def moveToBall(self):
        #Moves towards the ball while centering it
        #Need to change the state at some point
        if not self.ballVars['captured']:
            return self.move(y=.7,theta=(-float(self.ballVars['location'].x)/2.0))
        else:
            self.state+=1
            self.debugPrint('Loading Ball')
            return self.stop()

    def collectBall(self):
        servoPub = rospy.Publisher('/hoverboard/PWMRaw',PWMRaw)
        rest = PWMRaw(channel=5, pwm=0)
        one  = PWMRaw(channel=5, pwm=2)
        two  = PWMRaw(channel=5, pwm=7)
        numPublishTimes = 10
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

    #####################
    #Landmark Detection/Distance Functions#
    #####################
    def checkForLandmark(self, landmarkVis):
        #Updates the state w/r/t the landmark Visibility
        if landmarkVis.visible == 1:
            self.landmarkVars['inView'] = True
        else:
            self.landmarkVars['inView'] = False

    def getDistanceToLandmark(self, landmarkLoc):
        height = landmarkLoc.height
        #Calculates the distance to landmark
        self.landmarkVars['visibleLandmark'] = landmarkLoc.code
        self.landmarkVars['landmarkX'] = (float(landmarkLoc.xtop)+float(landmarkLoc.xbottom))/2.0

        #Calculate actual distance using interpolated polynomial
        distance = self.landmarkVars['distance']
        if height != 0:
            distance = 7334.8 * math.pow(height, -0.996)
        self.landmarkVars['distance'] = distance
        #TODO: Figure out the actual distance to use here
        self.debugPrint(self.landmarkVars['distance'])
        if self.landmarkVars['distance'] < 175 and self.colorDict[self.getCurrentColor()] == landmarkLoc.code:
            self.landmarkVars['targetReached'] = True
        else:
            self.landmarkVars['targetReached'] = False

    #!The actual state machine functions for landmark!#
    def lookForLandmark(self, landmarkNum):
        #This is straight up copied from lab3 ballDozer
        if not self.landmarkVars['inView'] or (self.landmarkVars['visibleLandmark'] != \
                                               landmarkNum and self.landmarkVars['visibleLandmark'] != -1):
            return self.move(theta=(-15))
        else:
            self.state += 1
            return self.move()

    def moveTowardsLandmark(self):
        #Move to landmark while centering it based on where the landmark location is found on the image
        if not self.landmarkVars['targetReached']:
            return self.move(y=1, theta=(-(float(self.landmarkVars['landmarkX'])-180.0)/10.0))
        else:
            self.state += 1
            return self.move()

    def launch(self):
        #Used to launch the ball from the barrel
        firePower = 50
        launchPub = rospy.Publisher('/hoverboard/PWMRaw',PWMRaw)
        FIRE = PWMRaw(channel=1, pwm=20)
        for i in xrange(0,50):
            launchPub.publish(FIRE)
        FIRE = PWMRaw(channel=1, pwm=firePower)
        for i in xrange(0,20000):
            launchPub.publish(FIRE)
        self.state += 1

        return self.move()
    def stopLaunching(self):
        #Stop firing the thruster
        launchPub = rospy.Publisher('/hoverboard/PWMRaw',PWMRaw)
        OHGODSTOPFIRING = PWMRaw(channel=1, pwm=20)
        launchPub.publish(OHGODSTOPFIRING)
        self.state += 1
        return self.move()


if __name__ == '__main__':
    rospy.init_node('BallCleaner')
    try:
        ne = BallCleaner()
        rospy.spin()
    except rospy.ROSInterruptException: pass
