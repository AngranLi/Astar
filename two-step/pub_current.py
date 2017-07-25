#!/usr/bin/env python

import random
import rospy
from geometry_msgs.msg import PoseStamped

'''
    This publishes a simulated current point PoseStamped every 10 seconds.
    The point is redefined randomly at certain frequancy.
'''

class start_pub():

    def __init__(self):
        markerPub = rospy.Publisher('/UAV_1/pose', PoseStamped, queue_size=10)
        rospy.Subscriber('/UAV_1/pose', PoseStamped, self.callback)
        rospy.init_node('pp_pub', anonymous=True)

        rate = rospy.Rate(1)

        self.current_point = PoseStamped()
        self.current_point.header.frame_id = "path_planner"
        self.current_point.header.stamp    = rospy.get_rostime()

        self.current_point.pose.position.x = 0.5
        self.current_point.pose.position.y = 0.5
        self.current_point.pose.position.z = 0.5
        self.current_point.pose.orientation.w = 1.0

        #a counter so the location changes

        self.counterMax = 2; # waiting time = counterMax*rospy.Rate
        self.counter = self.counterMax;

        while not rospy.is_shutdown():

            self.counter = self.counter + 1;

            if (self.counter > self.counterMax):

                markerPub.publish(self.current_point)

                self.counter = 0

                self.current_point.pose.position.x += 0.1 +random.uniform(-0.01, 0.01)
                self.current_point.pose.position.y += 0.1 +random.uniform(-0.01, 0.01)
                self.current_point.pose.position.z += 0.075 +random.uniform(-0.01, 0.01)
                self.current_point.pose.orientation.w = 1.0

            rate.sleep()

    def callback(self, data):
        #data is of type Marker
        #lets print some information
        print "Current position received!"
        print data.pose.position.x, data.pose.position.y, data.pose.position.z

start_pub()
