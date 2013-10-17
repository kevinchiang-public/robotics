#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('lab2')
from lab2.msg import irData,Range

class Tangent():
    self.debug=float(rospy.get_param('~debug','0'))
    rospy.Subscriber('/irData', irData, self.irCallback)
    rospy.Subscriber('/rangeInfo',Range, self.rangeCallback)

    def rangeCallback(self, ranges):
        leftIR = ranges.leftDistanceCM

    def irCallback(self,ir):
        pass



if __name__ == '__main__':
    rospy.init_node('tangent')
    try:
        ne = Tangent()
        rospy.spin()
    except rospy.ROSInterruptException: pass
