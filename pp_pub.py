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

class pp_pub():

    def __init__(self):
        markerPub = rospy.Publisher('pp_request', Marker, queue_size=10)
        rospy.Subscriber("pp_request", Marker, self.callback)
        rospy.init_node('pp_pub', anonymous=True)

        rate = rospy.Rate(1)

        self.pp_request = Marker()
        self.pp_request.header.frame_id = "path_planner"
        self.pp_request.header.stamp    = rospy.get_rostime()
        self.pp_request.ns = "path_planner"
        self.pp_request.id = 0
        self.pp_request.type = 8 # point
        self.pp_request.action = 0
        self.pp_request.pose.position.x = 0.0
        self.pp_request.pose.position.y = 0.0
        self.pp_request.pose.position.z = 0.0
        self.pp_request.pose.orientation.x = 0
        self.pp_request.pose.orientation.y = 0
        self.pp_request.pose.orientation.z = 0
        self.pp_request.pose.orientation.w = 1.0
        self.pp_request.scale.x = 1.0
        self.pp_request.scale.y = 1.0
        self.pp_request.scale.z = 1.0

        self.pp_request.color.r = 0.0
        self.pp_request.color.g = 1.0
        self.pp_request.color.b = 0.0
        self.pp_request.color.a = 1.0

        self.pp_request.lifetime = rospy.Duration(0)

        p0 = Point()
        p0.x = 0.0
        p0.y = 0.0
        p0.z = 0.0

        p1 = Point()
        p1.x = 3.0
        p1.y = 3.0
        p1.z = 3.0

        self.pp_request.points.append(p0)
        self.pp_request.points.append(p1)

        self.pp_request.text = "0"

        #a counter so the location changes

        self.counterMax = 10; # waiting time = counterMax*rospy.Rate
        self.counter = self.counterMax;

        while not rospy.is_shutdown():

            self.counter = self.counter + 1;

            if (self.counter > self.counterMax):

                markerPub.publish(self.pp_request)

                self.counter = 0

                p0 = Point()
                p0.x = 0.0
                p0.y = 0.0
                p0.z = 0.0

                p1 = Point()
                p1.x = random.uniform(1, 6)
                p1.y = random.uniform(1, 3)
                p1.z = random.uniform(1, 3)

                self.pp_request.points[0] = p0
                self.pp_request.points[1] = p1


            #print "sending marker", self.pp_request
            rate.sleep()

    def callback(self, data):
        #data is of type Marker
        #lets print some information
        print "Data received!"
        print "point 0: ", data.points[0].x, ", ", data.points[0].y, ", ", data.points[0].z
        print "point 1: ", data.points[1].x, ", ", data.points[1].y, ", ", data.points[1].z
        print "text :   ", data.text

pp_pub()
