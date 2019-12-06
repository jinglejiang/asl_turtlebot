#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose2D
import tf


class FruitVis():
    def __init__(self):
        rospy.init_node('fruit_marker_node', anonymous=True)

    def run(self):
        vis_pub = rospy.Publisher('marker_topic', Marker, queue_size=10)

        rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            fruits = rospy.get_param('fruit_locations', {})

            if not fruits:
                continue

            i = 0

            for f in fruits:
                marker = Marker()

                marker.header.frame_id = "map"
                marker.header.stamp = rospy.Time()

                # IMPORTANT: If you're creating multiple markers,
                #            each need to have a separate marker ID.
                marker.id = i

                marker.type = 2  # sphere

                marker.pose.position.x = self.x_g
                marker.pose.position.y = self.y_g
                marker.pose.position.z = 0

                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0

                marker.scale.x = .2
                marker.scale.y = .2
                marker.scale.z = .2

                marker.color.a = 1.0  # Don't forget to set the alpha!
                marker.color.r = i / 5.0
                marker.color.g = 1 - 1 / 5.0
                marker.color.b = 0.0

                i = i + 1

                vis_pub.publish(marker)

            print('Published marker!')

            rate.sleep()


if __name__ == '__main__':
    fruits = FruitVis()
    fruits.run()
