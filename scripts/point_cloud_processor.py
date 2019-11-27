#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import ros_numpy
from collections import Counter
import numpy as np


ZMIN = 0.0
ZMAX = 0.1

thetaleft = 0
thetaright = np.pi/4.0

class PCProcessor:

    def __init__(self):
        rospy.init_node('goal_pose_commander', anonymous=True)
        # initialize variables
        # command pose for controller
        rospy.Subscriber('/velodyne_points', PointCloud2, self.velodyne_callback_distance_measure)
        
    def velodyne_callback_noop(self, msg):
        """ callback for a pose goal sent through rviz """
        rospy.loginfo("velodyne recieved!")
        print ("header is", msg.header)
        print ("height is", msg.height)
        print ("width is", msg.width)
        print ("point_step is", msg.point_step)
        print ("row_step is", msg.row_step)

        data = ros_numpy.numpify(msg)
        # print data
        # print data.shape
        print ("x range: ", min(data['x']), max(data['x']))
        print ("y range: ", min(data['y']), max(data['y']))
        print ("z range: ", min(data['z']), max(data['z']))

        print type(data)
        print data.shape
        print type(data['x'])
        print data['x'].shape
        print dir(data)
        # tmp = np.rint(data['x']*100)
        # print ("x distribution", Counter(tmp))
        print "\n\n\n"
        

    def velodyne_callback_distance_measure(self, msg):
        data = ros_numpy.numpify(msg)
        filter_idx = np.logical_and(ZMIN<data['z'],  data['z']<ZMAX)
        print "{} out of {} fall inside ZMIN, ZMAX".format(np.sum(filter_idx), data.shape)
        data = data[filter_idx]

        angle = np.arctan2(data['y'], data['x']) + np.pi
        if thetaleft<thetaright:
            angle_idx = np.logical_and(thetaleft<angle,  angle<thetaright)
        else:
            angle_idx = np.logical_and(thetaright<angle,  angle<thetaleft)


        print "{} out of {} fall inside thetaleft, theteright".format(np.sum(angle_idx), data.shape)
        data = data[angle_idx]
        

        dist = np.sqrt(data['x']**2+data['y']**2+data['z']**2)
        print "distance min:{}, max:{}, median:{}".format(min(dist), max(dist), np.median(dist))


    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
        


if __name__ == '__main__':
    pc = PCProcessor()
    # pc.run()
    rospy.spin()
