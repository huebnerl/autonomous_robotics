#!/usr/bin/env python

from __future__ import print_function
from std_msgs.msg import Bool
from RoboPoint import RoboPoint
from RoboPoint import RoboMove
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import uarm_msgs.msg


class RoboArm:

    def __init__(self, robo_number, robo_move):
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        self.subTopic = '/sub_topic_robo_arm_' + str(robo_number)
        rospy.Subscriber(self.subTopic, Bool, self.move_a_stone)

        self.pubTopic = '/pub_topic_robo_arm_' + str(robo_number)
        self.pub = rospy.Publisher(self.pubTopic, Bool, queue_size=1)

        self.robo_number = robo_number
        self.node_name = 'roboarm' + str(robo_number)
        rospy.init_node(self.node_name, anonymous=True)

        self.spoint1 = robo_move.spoint1
        self.point1 = robo_move.point1
        self.spoint2 = robo_move.spoint2
        self.point2 = robo_move.point2

    def move_a_stone(self, data):
        print("lets move")
        self.pick_a_stone()
        self.drop_a_stone()
        self.publishFinished()

    def publishFinished(self):
        self.pub.publish(True)

    def pick_a_stone(self):
        # move
        moveClient = actionlib.SimpleActionClient('uarm_move', uarm_msgs.msg.uarm_move_dual_robotsAction)
        pumpClient = actionlib.SimpleActionClient('uarm_pump', uarm_msgs.msg.uarm_pump_dual_robotsAction)

        #secPoint
        moveClient.wait_for_server()
        goal = uarm_msgs.msg.uarm_move_dual_robotsGoal(x=self.spoint2.x, y=self.spoint2.y, z=self.spoint2.z, w=self.spoint2.w, robot_number=self.robo_number)
        moveClient.send_goal(goal)
        moveClient.wait_for_result()
        
        #secPoint
        moveClient.wait_for_server()
        goal = uarm_msgs.msg.uarm_move_dual_robotsGoal(x=self.spoint1.x, y=self.spoint1.y, z=self.spoint1.z, w=self.spoint1.w, robot_number=self.robo_number)
        moveClient.send_goal(goal)
        moveClient.wait_for_result()

        if not moveClient.get_result():
            return moveClient.get_result()

        #point
        moveClient.wait_for_server()
        goal = uarm_msgs.msg.uarm_move_dual_robotsGoal(x=self.point1.x, y=self.point1.y, z=self.point1.z, w=self.point1.w, robot_number=self.robo_number)
        moveClient.send_goal(goal)
        moveClient.wait_for_result()

        if not moveClient.get_result():
            return moveClient.get_result()

        #pump
        pumpClient.wait_for_server()
        goal = uarm_msgs.msg.uarm_pump_dual_robotsGoal(pump=True, robot_number=self.robo_number)
        pumpClient.send_goal(goal)
        pumpClient.wait_for_result()
        
        if not pumpClient.get_result():
            return pumpClient.get_result()

        #secPoint
        moveClient.wait_for_server()
        goal = uarm_msgs.msg.uarm_move_dual_robotsGoal(x=self.spoint1.x, y=self.spoint1.y, z=self.spoint1.z, w=self.spoint1.w, robot_number=self.robo_number)
        moveClient.send_goal(goal)
        moveClient.wait_for_result()

        return moveClient.get_result()

    def drop_a_stone(self):  
        # move
        moveClient = actionlib.SimpleActionClient('uarm_move', uarm_msgs.msg.uarm_move_dual_robotsAction)
        pumpClient = actionlib.SimpleActionClient('uarm_pump', uarm_msgs.msg.uarm_pump_dual_robotsAction)

        #secPoint
        moveClient.wait_for_server()
        goal = uarm_msgs.msg.uarm_move_dual_robotsGoal(x=self.spoint2.x, y=self.spoint2.y, z=self.spoint2.z, w=self.spoint2.w, robot_number=self.robo_number)
        moveClient.send_goal(goal)
        moveClient.wait_for_result()
        
        if not moveClient.get_result():
            return moveClient.get_result()

        #Point
        moveClient.wait_for_server()
        goal = uarm_msgs.msg.uarm_move_dual_robotsGoal(x=self.point2.x, y=self.point2.y, z=self.point2.z, w=self.point2.w, robot_number=self.robo_number)
        moveClient.send_goal(goal)
        moveClient.wait_for_result()
        
        if not moveClient.get_result():
            return moveClient.get_result()

        #Pump
        pumpClient.wait_for_server()
        goal = uarm_msgs.msg.uarm_pump_dual_robotsGoal(pump=False, robot_number=self.robo_number)
        pumpClient.send_goal(goal)
        pumpClient.wait_for_result()
        
        if not pumpClient.get_result():
            return pumpClient.get_result()
        
        #secPoint
        moveClient.wait_for_server()
        goal = uarm_msgs.msg.uarm_move_dual_robotsGoal(x=self.spoint2.x, y=self.spoint2.y, z=self.spoint2.z, w=self.spoint2.w, robot_number=self.robo_number)
        moveClient.send_goal(goal)
        moveClient.wait_for_result()

        return moveClient.get_result()
