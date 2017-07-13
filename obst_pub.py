#!/usr/bin/env python

import random

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker

'''
    This publishes a Marker every 10 secons which has 2 points attached to it.
    The points are redefined randomly every 10 seconds
'''

class obst_pub():

    def __init__(self):
        markerPub = rospy.Publisher('obst_request', Marker, queue_size=10)
        rospy.Subscriber("obst_request", Marker, self.callback)
        rospy.init_node('obst_pub', anonymous=True)

        rate = rospy.Rate(1)

        self.obst_request = Marker()
        self.obst_request.header.frame_id = "path_planner"
        self.obst_request.header.stamp    = rospy.get_rostime()
        self.obst_request.ns = "path_planner"
        self.obst_request.id = 0
        self.obst_request.type = 8 # point
        self.obst_request.action = 0
        self.obst_request.pose.position.x = 0.0
        self.obst_request.pose.position.y = 0.0
        self.obst_request.pose.position.z = 0.0
        self.obst_request.pose.orientation.x = 0
        self.obst_request.pose.orientation.y = 0
        self.obst_request.pose.orientation.z = 0
        self.obst_request.pose.orientation.w = 1.0
        self.obst_request.scale.x = 1.0
        self.obst_request.scale.y = 1.0
        self.obst_request.scale.z = 1.0

        self.obst_request.color.r = 1.0
        self.obst_request.color.g = 1.0
        self.obst_request.color.b = 1.0
        self.obst_request.color.a = 1.0

        self.obst_request.lifetime = rospy.Duration(0)

        centre_point = Point()
        centre_point.x = 1.0
        centre_point.y = 1.0
        centre_point.z = 0.0

        self.obst_request.points.append(centre_point)

        self.obst_request.text = "0"

        #a counter so the location changes

        self.counterMax = 1; # waiting time = counterMax*rospy.Rate
        self.counter = self.counterMax;

        while not rospy.is_shutdown():

            self.counter = self.counter + 1;

            if (self.counter > self.counterMax):

                markerPub.publish(self.obst_request)

                self.counter = 0

                centre_point = Point()
                centre_point.x = random.uniform(1, 6)
                centre_point.y = random.uniform(1, 3)
                centre_point.z = 1.0

		# self.obst_request.text = "update 1"

                self.obst_request.points[0] = centre_point


            #print "sending marker", self.obst_request
            rate.sleep()

    def callback(self, data):
        #data is of type Marker
        #lets print some information
        print "data received"
        print "centre_point:\t", data.points[0].x, ", ", data.points[0].y, ", ", data.points[0].z
        print "text :\t", data.text

obst_pub()
