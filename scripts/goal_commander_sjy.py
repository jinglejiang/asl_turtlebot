#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose2D, PoseStamped
from visualization_msgs.msg import Marker
import tf
from std_msgs.msg import String

# if using gmapping, you will have a map frame. otherwise it will be odom frame
mapping = True

class GoalPoseCommander:

    def __init__(self):
        rospy.init_node('goal_pose_commander', anonymous=True)
        # initialize variables
        self.x_g = None
        self.y_g = None
        self.theta_g = None
        self.goals = []
        self.goal_pose_received = False
        self.trans_listener = tf.TransformListener()
        self.start_time = rospy.get_rostime()
        # command pose for controller
        self.nav_goal_publisher = rospy.Publisher('/cmd_nav', Pose2D, queue_size=10)
        self.marker_pub = rospy.Publisher('marker_topic', Marker, queue_size=10)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)
        rospy.Subscriber('/goal_reach', String, self.release_goal)
        #rospy.Subscriber('/goal_cancel', String, self.release_goal)
        
    def rviz_goal_callback(self, msg):
        """ callback for a pose goal sent through rviz """
        rospy.loginfo("rviz command received!")
        print("???")
        try:
            origin_frame = "/map" if mapping else "/odom"
            rospy.loginfo("getting frame")
            nav_pose_origin = self.trans_listener.transformPose(origin_frame, msg)
            x_g = nav_pose_origin.pose.position.x
            y_g = nav_pose_origin.pose.position.y
            quaternion = (
                    nav_pose_origin.pose.orientation.x,
                    nav_pose_origin.pose.orientation.y,
                    nav_pose_origin.pose.orientation.z,
                    nav_pose_origin.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            theta_g = euler[2]
            self.goals.append([x_g, y_g, theta_g])
            print(self.goals)
            if self.x_g is None:
                self.x_g = x_g
                self.y_g = y_g
                self.theta_g = theta_g
            self.start_time = rospy.get_rostime()

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    def release_goal(self, data):
        if self.goals:
            print("release_goal")
            self.goals.pop(0)
            if self.goals:
                goal = self.goals[0]
                self.x_g = goal[0]
                self.y_g = goal[1]
                self.theta_g = goal[2]
                self.publish_goal_pose()
            else:
                self.x_g = None
                self.y_g = None
                self.theta_g = None


    def publish_goal_pose(self):
        """ sends the current desired pose to the navigator """
        if self.x_g is not None:
            pose_g_msg = Pose2D()
            pose_g_msg.x = self.x_g
            pose_g_msg.y = self.y_g
            pose_g_msg.theta = self.theta_g
            self.nav_goal_publisher.publish(pose_g_msg)

    def marker(self):

        for i, goal in enumerate(self.goals):
            marker_x = goal[0]
            marker_y = goal[1]
            marker_theta = goal[2]

            quat = tf.transformations.quaternion_from_euler(0, 0, marker_theta)

            marker = Marker()

            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time()

            # IMPORTANT: If you're creating multiple markers,
            #            each need to have a separate marker ID.
            marker.id = i

            marker.type = 0  # sphere

            marker.pose.position.x = marker_x
            marker.pose.position.y = marker_y
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

            self.marker_pub.publish(marker)

    def loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            t = rospy.get_rostime()
            if (t - self.start_time).to_sec() < 0.11:
                self.publish_goal_pose()
                self.marker()
            rate.sleep()
        


if __name__ == '__main__':
    sup = GoalPoseCommander()
    try:
        sup.loop()

    except rospy.ROSInterruptException:
        pass        
    
