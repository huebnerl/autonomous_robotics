#!/usr/bin/env python

from conveyor.srv import Int16
from std_msgs.msg import Bool
from conveyor.msg import ConveyorState
import rospy

class Conveyor:

    def __init__(self):
        rospy.init_node('conveyorpy')
        rospy.Subscriber('/pub_ready_for_stone_2', Bool, self.start)
        rospy.Subscriber('/conveyor1/state', ConveyorState, self.ready)
        self.pub = rospy.Publisher('/sub_topic_robo_arm_2', Bool, queue_size=1)

        self.stone_in_front = False

    def ready(self, data):
        if(1 <= data.distance_sonic <= 4 and self.stone_in_front == False):
            self.stone_in_front = True
            self.pub.publish(True)
            print("conveyor ready")
        elif( data.distance_sonic > 7 ):
            self.stone_in_front = False

    def start(self, data):
        print(data)
        rospy.wait_for_service('/conveyor1/move_forward')
        try:
            conveyor = rospy.ServiceProxy('/conveyor1/move_forward', Int16)
            print("start conveyor")
            resp = conveyor(data=30)
            return resp.success
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


if __name__ == '__main__':
    belt = Conveyor()
    rospy.spin()