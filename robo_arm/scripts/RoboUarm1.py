#!usr/bin/env python

from __future__ import print_function
from RoboPoint import RoboPoint
from RoboPoint import RoboMove
from RoboUarm import RoboArm
from std_msgs.msg import Bool
import rospy

if __name__ == '__main__':
    try:
        spoint1 = RoboPoint(x=220, y=-40, z=174, w=90)
        point1 = RoboPoint(x=220, y=-40, z=168, w=90)
        spoint2 = RoboPoint(x=110, y=160, z=168, w=60)
        point2 = RoboPoint(x=110, y=160, z=46, w=60)
        roboMove1 = RoboMove(spoint1, point1, spoint2, point2)
        roboArm1 = RoboArm(robo_number=1, robo_move=roboMove1)

        roboArm1.move_a_stone(None)
        #pub = rospy.Publisher('/sub_topic_robo_arm_1', Bool, queue_size=10)
        #pub.publish(True)
        rospy.spin()
    except rospy.ROSInterruptException:
        print("program interrupted before completion" + sys.stderr)
        pass
