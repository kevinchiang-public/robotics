#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('lab3')
from lab3.msg import Movement

class Ball():
    def __init__(self):
        self.debug = float(rospy.get_param('~debug', '0'))

        #Stopping the hovercraft
        self.haltMove = Movement()
        self.haltMove.theta = 0
        self.haltMove.x = 0
        self.haltMove.y = 0
        self.haltMove.modType = 'Add'

    def stateMachine(self):
        pass

    #This finds the ball.  We are going to rotate (maybe 30 degrees, according to field of vision of camera), then cycle through colors to see if it detects one.
    def detectBall(self):
        pass

    def detectColor(self):
        pass
    
    def moveToBall(self):
        pass

    def collectBall(self):
        pass

    def loadBall(self):
        pass

    #Spins the ball
    def spin(self):
        move = Movement()
        move.modType = 'Add'
        move.theta = -15
        move.x = 0
        move.y = 0
        return move

if __name__=='__main__':
    rospy.init_node('BallMover')
    try:
        ne = Ball()
        rospy.spin()
    except rospy.ROSInterruptException: pass
