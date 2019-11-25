#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String
import tf


class TargetVis():
    def __init__(self):
        rospy.init_node('marker_node', anonymous=True)
        rospy.Subscriber('/cmd_nav', Pose2D, self.cmd_nav_callback)
        rospy.Subscriber('/goal_reach', String, self.release_goal)

        # goal state
        self.x_g = None
        self.y_g = None
        self.theta_g = None

        # goal state list
        self.goals = []

    def cmd_nav_callback(self, data):
        """
        loads in goal if different from current goal, 
        """
        if data.x != self.x_g or data.y != self.y_g or data.theta != self.theta_g:
            self.goals.append([data.x, data.y, data.theta])

    def release_goal(self):
        if self.goals:
            self.goals.pop(0)

    def run(self):
        vis_pub = rospy.Publisher('marker_topic', Marker, queue_size=10)

        rate = rospy.Rate(1)

        while not rospy.is_shutdown():

            for i, goal in enumerate(self.goals):
                self.x_g = goal[0]
                self.y_g = goal[1]
                self.theta_g = goal[2]

                quat = tf.transformations.quaternion_from_euler(0, 0, self.theta_g)

                marker = Marker()

                marker.header.frame_id = "map"
                marker.header.stamp = rospy.Time()

                # IMPORTANT: If you're creating multiple markers,
                #            each need to have a separate marker ID.
                marker.id = i

                marker.type = 0  # sphere

                marker.pose.position.x = self.x_g
                marker.pose.position.y = self.y_g
                marker.pose.position.z = 0

                marker.pose.orientation.x = quat[0]
                marker.pose.orientation.y = quat[1]
                marker.pose.orientation.z = quat[2]
                marker.pose.orientation.w = quat[3]

                marker.scale.x = 1
                marker.scale.y = .1
                marker.scale.z = .1

                marker.color.a = 1.0  # Don't forget to set the alpha!
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0

                vis_pub.publish(marker)

                print('Published marker!', i)

        rate.sleep()


if __name__ == '__main__':
    tv = TargetVis()
    tv.run()
