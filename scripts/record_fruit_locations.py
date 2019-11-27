#!/usr/bin/env python

"""record_fruit_locations.py: record locations of the fruits
if detected.
"""
import rospy
from gazebo_msgs.msg import ModelStates
from asl_turtlebot.msg import DetectedObject
import tf


FOOD_LABELS = ['banana', 'apple','sandwich', 'orange', 'broccoli', 'carrot', 'hot_dog', 'pizza', 'donut', 'cake', 'salad', 'bottle']
use_gazebo = rospy.get_param("sim")


class Record:
    """record fruit locations if detected"""

    def __init__(self):
        rospy.init_node('record_fruit_locations', anonymous=True)
        self.x = 0
        self.y = 0
        self.theta = 0
        self.trans_listener = tf.TransformListener()
        self.food_publisher_list = {}
        self.fruit_locations = {}
        for food in FOOD_LABELS:
            # if not self.food_publisher_list.has_key(food):
            self.food_publisher_list[food] = rospy.Subscriber('/detector/'+food, DetectedObject, self.fruit_detected_callback)
        if use_gazebo:
            rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
   
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
    
    def fruit_detected_callback(self, msg):
        dst = msg.distance
        fruit_name = msg.name
        self.fruit_locations[fruit_name] = (self.x, self.y, self.theta)
        rospy.set_param('fruit_locations', self.fruit_locations)


    def loop(self):
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
    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

if __name__ == "__main__":
    recorder = Record()
    recorder.run()




        
    

