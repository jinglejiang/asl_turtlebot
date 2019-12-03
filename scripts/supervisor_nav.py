#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, String, Int32MultiArray
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from asl_turtlebot.msg import DetectedObject
import tf
import math
from enum import Enum

# if sim is True/using gazebo, therefore want to subscribe to /gazebo/model_states\
# otherwise, they will use a TF lookup (hw2+)
use_gazebo = False#rospy.get_param("sim")

# if using gmapping, you will have a map frame. otherwise it will be odom frame
mapping = True #rospy.get_param("map")


# threshold at which we consider the robot at a location
POS_EPS = .1
THETA_EPS = .3
FRUIT_VICINITY = .5 # TODO change parameter. What is the unit used??

# time to stop at a stop sign
STOP_TIME = 10

# minimum distance from a stop sign to obey it
STOP_MIN_DIST = 3
FRUIT_STOP_MIN_DIST = 3

# time taken to cross an intersection
CROSSING_TIME = 3

# state machine modes, not all implemented
class Mode(Enum):
    IDLE = 1
    POSE = 2
    STOP = 3
    CROSS = 4
    NAV = 5
    MANUAL = 6
    FRUITSTOP = 7
    EXPLORE = 8


print "supervisor settings:\n"
print "use_gazebo = %s\n" % use_gazebo
print "mapping = %s\n" % mapping

class Supervisor:

    def __init__(self):
        rospy.init_node('turtlebot_supervisor_nav', anonymous=True)
        # initialize variables
        self.x = 0
        self.y = 0
        if use_gazebo:
            self.x = 3.35
            self.y = 2.4
        self.theta = 0
        self.home = (self.x, self.y, self.theta)
        self.mode = Mode.IDLE
        self.last_mode_printed = None
        self.trans_listener = tf.TransformListener()
        # command pose for controller
        self.pose_goal_publisher = rospy.Publisher('/cmd_pose', Pose2D, queue_size=10)
        # nav pose for controller
        self.nav_goal_publisher = rospy.Publisher('/cmd_nav', Pose2D, queue_size=10)
        # command vel (used for idling)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # subscribers
        # stop sign detector
        rospy.Subscriber('/detector/stop_sign', DetectedObject, self.stop_sign_detected_callback)


        # # any food (basically any interesting object othan than stop sign) detector
        # rospy.Subscriber('/detector/stop_sign', DetectedObject, self.stop_sign_detected_callback)


        # high-level navigation pose
        rospy.Subscriber('/nav_pose', Pose2D, self.nav_pose_callback)
        # if using gazebo, we have access to perfect state
        if use_gazebo:
            rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
        # we can subscribe to nav goal click
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)

        # get fruit locations list
        if use_gazebo:
            # gazebo hardcode simulation
            self.locations = {'apple':(3, 1, 0), 'banana':(1.5, 2.7, 0), 'cake':(3, 2, 0)}
        else:
            self.locations = rospy.get_param('fruit_locations')#{1:(3, 1, 0), 2:(1.5, 2.7, 0), 3:(3, 2, 0)}
        # subscribe cmd input containing fruits to deliver, "apple,banana,..."
        rospy.Subscriber('/delivery_request', String, self.fruits_cmd_callback)
            
    def gazebo_callback(self, msg):
        pose = msg.pose[msg.name.index("turtlebot3_burger")]
        twist = msg.twist[msg.name.index("turtlebot3_burger")]
        self.x = pose.position.x
        self.y = pose.position.y
        quaternion = (
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.theta = euler[2]

    def rviz_goal_callback(self, msg):
        """ callback for a pose goal sent through rviz """
        origin_frame = "/map" if mapping else "/odom"
        print("rviz command received!")
        try:
            
            nav_pose_origin = self.trans_listener.transformPose(origin_frame, msg)
            self.x_g = nav_pose_origin.pose.position.x
            self.y_g = nav_pose_origin.pose.position.y
            quaternion = (
                    nav_pose_origin.pose.orientation.x,
                    nav_pose_origin.pose.orientation.y,
                    nav_pose_origin.pose.orientation.z,
                    nav_pose_origin.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            self.theta_g = euler[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        self.mode = Mode.NAV

    def nav_pose_callback(self, msg):
        self.x_g = msg.x
        self.y_g = msg.y
        self.theta_g = msg.theta
        self.mode = Mode.NAV
    
    # def record_fruit_locations_callback(self, msg):
    #     """
    #     Record fruit locations when msg is published.
    #     """
    #     self.num_fruits = msg.layout.dim[0]
    #     self.fruits = msg.data # int32 array (list) of dimension (n, 3)
    #     # we need to define the structure for msg to include fruit name and location
    #     # self.locations: a dict of tuples, self.locations[fruit_name] = x, y, theta
    #     self.locations = dict()
    #     for name, x, y in fruits:
    #         self.locations[name] = (x, y, 0.) # always set goal theta to be 0; TODO unsure about this

    def stop_sign_detected_callback(self, msg):
        """ callback for when the detector has found a stop sign. Note that
        a distance of 0 can mean that the lidar did not pickup the stop sign at all """

        # distance of the stop sign
        dist = msg.distance

        # if close enough and in nav mode, stop
        if dist > 0 and dist < STOP_MIN_DIST and self.mode == Mode.NAV:
            self.init_stop_sign()
    
    def fruits_cmd_callback(self, msg):
        """
        Rui: delivery requests sent, add the requested fruits to self.goals.
        Then, start navigating to the first fruit.
        """

        fruits_to_collect = msg.data.split(',')
        print(fruits_to_collect)
        #self.num_goals = len(fruits_to_collect) #msg.layout.dim[0] # fruits to collect
        # fruits_to_collect = msg.data # array of fruit names (string)
        self.mode = Mode.NAV
        self.goals = []
        self.goal_names = []
        for fruit_name in fruits_to_collect:
            if self.locations.has_key(fruit_name):
                self.goals.append(self.locations[fruit_name]) # a tuple (x, y, theta)
                self.goal_names.append(fruit_name)
            else:
                rospy.loginfo("location of "+fruit_name+" not loaded in exploration")
        # set a fruit as current goal
        # self.x_g, self.y_g, self.theta_g = self.goals.pop()
        self.x_g, self.y_g, self.theta_g = self.goals[-1]
        # rospy.loginfo(self.goals)

    def go_to_pose(self):
        """ sends the current desired pose to the pose controller """

        pose_g_msg = Pose2D()
        pose_g_msg.x = self.x_g
        pose_g_msg.y = self.y_g
        pose_g_msg.theta = self.theta_g

        self.pose_goal_publisher.publish(pose_g_msg)

    def nav_to_pose(self):
        """ sends the current desired pose to the naviagtor """

        nav_g_msg = Pose2D()
        nav_g_msg.x = self.x_g
        nav_g_msg.y = self.y_g
        nav_g_msg.theta = self.theta_g

        self.nav_goal_publisher.publish(nav_g_msg)

    def stay_idle(self):
        """ sends zero velocity to stay put """

        vel_g_msg = Twist()
        self.cmd_vel_publisher.publish(vel_g_msg)

    def close_to_home(self):
        """ checks if the robot is at a pose within some threshold """
        x, y, theta = self.home
        return (abs(x-self.x)<POS_EPS and abs(y-self.y)<POS_EPS and abs(theta-self.theta)<THETA_EPS)

    def close_to_some_fruit(self):
        """
        returns: flag, true or false; fruit_name
        when the bot is getting close to a fruit requested
        """
        for loc, name in zip(self.goals, self.goal_names):
            x, y, theta = loc
            if abs(x-self.x) < FRUIT_VICINITY and abs(y-self.y) < FRUIT_VICINITY:
                return True, name
        return False, None

    def init_stop_sign(self):
        """ initiates a stop sign maneuver """

        self.stop_sign_start = rospy.get_rostime()
        self.mode = Mode.STOP
    
    def init_stop_fruit(self, fruit_name):
        """
        Rui: stop for a fruit. Change goal for next step in the mean time.
        """
        self.stop_fruit_start = rospy.get_rostime()
        self.mode = Mode.FRUITSTOP
        rospy.loginfo("I find {}!! I will stop for a bit".format(fruit_name))

        # now we got figure out if the fruit we are stopping at is our current goal or not
        closest_fruit_location = self.locations[fruit_name]
        if closest_fruit_location[0] == self.x_g and closest_fruit_location[1] == self.y_g:
            # this fruit is exactly the fruit we are looking for
            # directly go to next goal
            self.goals.pop()
            if len(self.goals) > 0:
                self.x_g, self.y_g, self.theta_g = self.goals[-1]
            else:
                self.x_g, self.y_g, self.theta_g = self.home # return to home location
        else:
            # remove this fruit from goals
            for each_goal in self.goals:
                if each_goal[0] == closest_fruit_location[0] and each_goal[1] == closest_fruit_location[1]:
                    self.goals.remove(each_goal)
                    break
        
            # do not need to change current goal, because it is not reached!

    def has_stopped(self):
        """ checks if stop sign maneuver is over """

        return (self.mode == Mode.STOP and (rospy.get_rostime()-self.stop_sign_start)>rospy.Duration.from_sec(STOP_TIME))

    def has_fruit_stopped(self):
        """ checks if stop fruit maneuver is over """

        return (self.mode == Mode.FRUITSTOP and (rospy.get_rostime()-self.stop_fruit_start)>rospy.Duration.from_sec(STOP_TIME))

    def init_crossing(self):
        """ initiates an intersection crossing maneuver """

        self.cross_start = rospy.get_rostime()
        self.mode = Mode.CROSS

    def has_crossed(self):
        """ checks if crossing maneuver is over """

        return (self.mode == Mode.CROSS and (rospy.get_rostime()-self.cross_start)>rospy.Duration.from_sec(CROSSING_TIME))

    def loop(self):
        """ the main loop of the robot. At each iteration, depending on its
        mode (i.e. the finite state machine's state), if takes appropriate
        actions. This function shouldn't return anything """

        if not use_gazebo:
            try:
                origin_frame = "/map" if mapping else "/odom"
                (translation,rotation) = self.trans_listener.lookupTransform(origin_frame, '/base_footprint', rospy.Time(0))
                self.x = translation[0]
                self.y = translation[1]
                euler = tf.transformations.euler_from_quaternion(rotation)
                self.theta = euler[2]
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

        # logs the current mode
        if not(self.last_mode_printed == self.mode):
            rospy.loginfo("Current Mode: %s", self.mode)
            self.last_mode_printed = self.mode

        # checks wich mode it is in and acts accordingly
        if self.mode == Mode.EXPLORE:
            pass

        elif self.mode == Mode.IDLE:
            # send zero velocity
            self.stay_idle()

        elif self.mode == Mode.STOP:
            # at a stop sign
            if self.has_stopped():
                self.init_crossing()
            else:
                self.stay_idle() # zero velocity
        
        elif self.mode == Mode.FRUITSTOP:
            # at a stop sign
            if self.has_fruit_stopped():
                # self.mode = Mode.CROSS
                self.init_crossing()
            else:
                self.stay_idle() # zero velocity

        elif self.mode == Mode.CROSS:
            # crossing an intersection
            if self.has_crossed():
                self.mode = Mode.NAV
            else:
                self.nav_to_pose()

        elif self.mode == Mode.NAV:
            if self.x_g == self.home[0] and self.y_g == self.home[1] and self.close_to_home:
                self.mode = Mode.IDLE
                rospy.loginfo("Mission Acoomplished! I am back home! I had lot of fun!")
            else:
                flag, fruit_name = self.close_to_some_fruit()
                if flag:
                    # Rui: stop for this fruit
                    self.init_stop_fruit(fruit_name)
                else:
                    self.nav_to_pose()

        else:
            raise Exception('This mode is not supported: %s'
                % str(self.mode))

    def run(self):
        # explore
        rate = rospy.Rate(10) # 10 Hz
        # TODO add explore function
        # explore() # ending condition: receives some command
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

if __name__ == '__main__':
    sup = Supervisor()
    sup.run()
