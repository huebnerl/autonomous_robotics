#!usr/bin/env python

from __future__ import print_function
from RoboPoint import RoboPoint
from RoboPoint import RoboMove
from RoboUarm import RoboArm
import rospy

if __name__ == '__main__':
    try:
        spoint1 = RoboPoint(x=120, y=-160, z=160, w=0)
        point1 = RoboPoint(x=120, y=-160, z=50, w=0)
        spoint2 = RoboPoint(x=160, y=160, z=174, w=0)
        point2 = RoboPoint(x=160, y=160, z=165, w=0)
        roboMove2 = RoboMove(spoint1, point1, spoint2, point2)
        roboArm2 = RoboArm(robo_number=2, robo_move=roboMove2)

        rospy.spin()
    except rospy.ROSInterruptException:
        print("program interrupted before completion" + sys.stderr)
        pass
