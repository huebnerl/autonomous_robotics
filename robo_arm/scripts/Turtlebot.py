#!/usr/bin/env python

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseActionResult
from tf import TransformListener
import tf
import rospy

class TurtleBot:

    def __init__(self):
        self.goals_to_arm1 = 0
        self.goals_to_arm2 = 0
        self.current_move = 'move_to_arm2'
        rospy.init_node('turtlebotpy')
        rospy.Subscriber('/pub_topic_robo_arm_1', Bool, self.start_move_to_arm2)
        rospy.Subscriber('/pub_topic_robo_arm_2', Bool, self.start_move_to_arm1)

        self.pub_turtle_arm2 = rospy.Publisher('/pub_turtle_at_arm_2', Bool, queue_size=1)
        rospy.Subscriber('/sub_turtle_fine_arm_2', Bool, self.fine_positioning)
        self.pub = rospy.Publisher('/sub_topic_robo_arm_1', Bool, queue_size=1)

        rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.move)
        self.pub_move = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.pub_fine = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.tf_listener = TransformListener()
        rospy.sleep(5)

    def start_move_to_arm1(self, data):
        self.move_to_arm1(None)

    def start_move_to_arm2(self, data):
        self.move_to_arm2(None)

    def marker_can_be_seen(self):
        rospy.sleep(10)
        print("looking for marker")
        try:
            (trans,rot) = self.tf_listener.lookupTransform('raspicam', 'fiducial_2', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return False
        return True

    def fine_positioning(self, data):
        twist = Twist()
        if self.marker_can_be_seen():
            # move foreward
            twist.linear.x = 0.05
            print("move foreward")
            self.pub_fine.publish(twist)
            self.pub_turtle_arm2.publish(True)
            return

        twist.angular.z = 0.5
        self.pub_fine.publish(twist)
        print("turn left")
        rospy.sleep(10)
        if self.marker_can_be_seen():
            self.pub_turtle_arm2.publish(True)
            return
            
        twist.angular.z = -1.0
        self.pub_fine.publish(twist)        
        print("turn right")
        rospy.sleep(10)
        if self.marker_can_be_seen():
            self.pub_turtle_arm2.publish(True)
            return

        self.pub_turtle_arm2.publish(True)
        return
        


    def move(self, data):
        if self.current_move == 'move_to_arm1':
            self.move_to_arm1(data)
        elif self.current_move == 'move_to_arm2':
            self.move_to_arm2(data)

    def move_to_arm2(self, data):
        self.current_move = 'move_to_arm2'
        if data != None and data.status.status == 3:
            self.goals_to_arm2 += 1
            print("Goal reached")

        if self.goals_to_arm2 == 0:    
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.pose.position.x = -0.13
            goal.pose.position.y = 0.0
            goal.pose.position.z = 0.0
            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = 0.0
            goal.pose.orientation.z = -1.0
            goal.pose.orientation.w = 1.0
            self.pub_move.publish(goal)

        if self.goals_to_arm2 == 1:    
            goal = PoseStamped()
            goal.header.frame_id = 'base_link'
            goal.pose.position.x = 0.0
            goal.pose.position.y = 0.0
            goal.pose.position.z = 0.0
            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = 0.0
            goal.pose.orientation.z = 1.0
            goal.pose.orientation.w = 1.0
            self.pub_move.publish(goal)
        
        if self.goals_to_arm2 == 2:    
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.pose.position.x = 0.45
            goal.pose.position.y = 0.0
            goal.pose.position.z = 0.0
            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = 0.0
            goal.pose.orientation.z = 0.0
            goal.pose.orientation.w = 1.0
            self.pub_move.publish(goal)

        if self.goals_to_arm2 == 3:    
            goal = PoseStamped()
            goal.header.frame_id = 'base_link'
            goal.pose.position.x = 0.0
            goal.pose.position.y = 0.0
            goal.pose.position.z = 0.0
            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = 0.0
            goal.pose.orientation.z = -1.0
            goal.pose.orientation.w = 1.0
            self.pub_move.publish(goal)

        if self.goals_to_arm2 == 4:    
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.pose.position.x = 0.46
            goal.pose.position.y = -0.25
            goal.pose.position.z = 0.0
            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = 0.0
            goal.pose.orientation.z = -1.0
            goal.pose.orientation.w = 1.0
            self.pub_move.publish(goal)

        if self.goals_to_arm2 == 5:
            self.goals_to_arm2 = 0
            self.pub_turtle_arm2.publish(True)

    
    def move_to_arm1(self, data):
        self.current_move = 'move_to_arm1'
        if data != None and data.status.status == 3:
            self.goals_to_arm1 += 1
            print("Goal reached")

        if self.goals_to_arm1 == 0:    
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.pose.position.x = -0.13
            goal.pose.position.y = 0.0
            goal.pose.position.z = 0.0
            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = 0.0
            goal.pose.orientation.z = 0.0
            goal.pose.orientation.w = 1.0
            self.pub_move.publish(goal)

        if self.goals_to_arm1 == 1:  
            goal = PoseStamped()
            goal.header.frame_id = 'base_link'
            goal.pose.position.x = 0.0
            goal.pose.position.y = 0.0
            goal.pose.position.z = 0.0
            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = 0.0
            goal.pose.orientation.z = -1.0
            goal.pose.orientation.w = 1.0
            self.pub_move.publish(goal)

        if self.goals_to_arm1 == 2:  
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.pose.position.x = -0.12
            goal.pose.position.y = -0.33
            goal.pose.position.z = 0.0
            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = 0.0
            goal.pose.orientation.z = -1.0
            goal.pose.orientation.w = 1.0
            self.pub_move.publish(goal)
        
        if self.goals_to_arm1 == 3:
            self.goals_to_arm1 = 0

    
    def start_carrying(self, data):
        pass


if __name__ == '__main__':
    turtle = TurtleBot()
    #turtle.move(None)
    #turtle.fine_positioning(None)
    rospy.spin()
