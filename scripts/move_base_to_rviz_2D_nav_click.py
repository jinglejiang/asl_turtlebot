#!/usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose2D
import tf
import actionlib

# this also serves as a fake move_base server
'''
def handle_MoveBaseActionGoal(data):
    print("handle_MoveBaseActionGoal received!")
    nav_goal_pub = rospy.Publisher('/cmd_nav', PoseStamped, queue_size=10)
        
    pose2D = Pose2D()
    pose2D.x =  data.pose.position.x
    pose2D.y =  data.pose.position.y
    self.nav_goal_pub(pose2D)


def MoveBaseTo2DNavClickAndFakeServer():
    rospy.init_node('fake_move_base_server')
    s = rospy.Service('move_base', MoveBaseActionGoal, handle_MoveBaseActionGoal)
    print "Fake move_base ready!"
    rospy.spin()

if __name__ == '__main__':
    MoveBaseTo2DNavClickAndFakeServer()
'''


#! /usr/bin/env python


class FakeMmoveBase:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('move_base', MoveBaseAction, self.execute, False)
    self.server.start()
    self.nav_goal_pub = rospy.Publisher('/cmd_nav', Pose2D, queue_size=10)

  def execute(self, goal):
    # Do lots of awesome groundbreaking robot stuff here
    # self.server.set_succeeded()
    print "recieved goal=" + str(goal)
    print "I will publish this to '/cmd_nav"
    pose2D = Pose2D()
    pose2D.x =  goal.target_pose.pose.position.x
    pose2D.y =  goal.target_pose.pose.position.y
    
    quaternion = (
            goal.target_pose.pose.orientation.x,
            goal.target_pose.pose.orientation.y,
            goal.target_pose.pose.orientation.z,
            goal.target_pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    pose2D.theta = euler[2]
    
    self.nav_goal_pub.publish(pose2D)
    print "published goal=" + str(pose2D)
    self.server.set_succeeded()

if __name__ == '__main__':
  rospy.init_node('move_base')
  server = FakeMmoveBase()
  rospy.spin()