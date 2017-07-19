#!/usr/bin/env python

import random
import rospy
from geometry_msgs.msg import PoseStamped

'''
    This publishes a simulated centre point of obstacle PoseStamped every 10 seconds.
    The point is redefined randomly at certain frequancy.
'''

class obst_pub():

    def __init__(self):
        markerPub = rospy.Publisher('obst_request', PoseStamped, queue_size=10)
        rospy.Subscriber("obst_request", PoseStamped, self.callback)
        rospy.init_node('obst_pub', anonymous=True)

        rate = rospy.Rate(1)

        self.obst_request = PoseStamped()
        self.obst_request.header.frame_id = "path_planner"
        self.obst_request.header.stamp    = rospy.get_rostime()

        self.obst_request.pose.position.x = 0.0
        self.obst_request.pose.position.y = 0.0
        self.obst_request.pose.position.z = 0.0
        self.obst_request.pose.orientation.w = 1.0

        #a counter so the location changes

        self.counterMax = 10; # waiting time = counterMax*rospy.Rate
        self.counter = self.counterMax;

        while not rospy.is_shutdown():

            self.counter = self.counter + 1;

            if (self.counter > self.counterMax):

                markerPub.publish(self.obst_request)

                self.counter = 0

                self.obst_request.pose.position.x = random.uniform(0,5)
                self.obst_request.pose.position.y = random.uniform(0,5)
                self.obst_request.pose.position.z = random.uniform(0,5)

            rate.sleep()

    def callback(self, data):
        #data is of type Marker
        #lets print some information
        print "Obstacle received!"
        print "centre_point:\t", data.pose.position.x, ", ", data.pose.position.y, ", ", data.pose.position.z

obst_pub()
