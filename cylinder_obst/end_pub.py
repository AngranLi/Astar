#!/usr/bin/env python

import random
import rospy
from geometry_msgs.msg import PoseStamped

'''
    This publishes a target point PoseStamped every 10 seconds.
    The point is redefined randomly at certain frequancy.
'''

class end_pub():

    def __init__(self):
        markerPub = rospy.Publisher('end_request', PoseStamped, queue_size=10)
        rospy.Subscriber("end_request", PoseStamped, self.callback)
        rospy.init_node('pp_pub', anonymous=True)

        rate = rospy.Rate(1)

        self.target_point = PoseStamped()
        self.target_point.header.frame_id = "path_planner"
        self.target_point.header.stamp    = rospy.get_rostime()

        self.target_point.pose.position.x = 2.0
        self.target_point.pose.position.y = 2.0
        self.target_point.pose.position.z = 2.0
        self.target_point.pose.orientation.w = 1.0

        #a counter so the location changes

        self.counterMax = 10; # waiting time = counterMax*rospy.Rate
        self.counter = self.counterMax;

        while not rospy.is_shutdown():

            self.counter = self.counter + 1;

            if (self.counter > self.counterMax):

                markerPub.publish(self.target_point)

                self.counter = 0

                self.target_point.pose.position.x = random.uniform(2,3)
                self.target_point.pose.position.y = random.uniform(2,3)
                self.target_point.pose.position.z = random.uniform(2,3)
                self.target_point.pose.orientation.w = 1.0

            rate.sleep()

    def callback(self, data):
        #data is of type Marker
        #lets print some information
        print "Destination position received!"
        print data.pose.position.x, data.pose.position.y, data.pose.position.z

end_pub()
