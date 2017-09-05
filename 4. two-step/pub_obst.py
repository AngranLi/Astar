#!/usr/bin/env python

import random
import rospy
from geometry_msgs.msg import PoseStamped

'''
    This publishes a simulated centre point of obstacle PoseStamped every 10 seconds.
    The point is redefined randomly at certain frequancy.
'''

class pub_obst():

    def __init__(self):
        UAV1_Pub = rospy.Publisher('/UAV_2/pose', PoseStamped, queue_size=10)
        UGV1_Pub = rospy.Publisher('/UAV_3/pose', PoseStamped, queue_size=10)
        person1_Pub = rospy.Publisher('/UAV_4/pose', PoseStamped, queue_size=10)
        rospy.Subscriber("/UAV_2/pose", PoseStamped, self.callback_UAV_2)
        rospy.Subscriber("/UAV_3/pose", PoseStamped, self.callback_UAV_3)
        rospy.Subscriber("/UAV_4/pose", PoseStamped, self.callback_UAV_4)
        rospy.init_node('obst_pub', anonymous=True)

        rate = rospy.Rate(1)

        self.UAV1 = PoseStamped()
        self.UAV1.header.frame_id = "path_planner"
        self.UAV1.header.stamp    = rospy.get_rostime()

        self.UAV1.pose.position.x = 2.0
        self.UAV1.pose.position.y = 2.0
        self.UAV1.pose.position.z = 1.0
        self.UAV1.pose.orientation.w = 1.0


        self.UGV1 = PoseStamped()
        self.UGV1.header.frame_id = "path_planner"
        self.UGV1.header.stamp    = rospy.get_rostime()

        self.UGV1.pose.position.x = 2.0
        self.UGV1.pose.position.y = 2.0
        self.UGV1.pose.position.z = 1.0
        self.UGV1.pose.orientation.w = 1.0

        self.person1 = PoseStamped()
        self.person1.header.frame_id = "path_planner"
        self.person1.header.stamp    = rospy.get_rostime()

        self.person1.pose.position.x = 2.0
        self.person1.pose.position.y = 2.0
        self.person1.pose.position.z = 1.0
        self.person1.pose.orientation.w = 1.0

        # a counter so the location changes
        self.counterMax = 4; # waiting time = counterMax*rospy.Rate
        self.counter = self.counterMax;

        while not rospy.is_shutdown():
            self.counter = self.counter + 1;
            if (self.counter > self.counterMax):

                self.counter = 0
                UAV1_Pub.publish(self.UAV1)
                self.UAV1.pose.position.x = random.uniform(0.25, 3.75)
                self.UAV1.pose.position.y = random.uniform(0.25, 3.75)
                self.UAV1.pose.position.z = random.uniform(1.0, 2.5)

                UGV1_Pub.publish(self.UGV1)
                self.UGV1.pose.position.x = random.uniform(1.5, 3.5)
                self.UGV1.pose.position.y = random.uniform(1.5, 3.5)
                self.UGV1.pose.position.z = random.uniform(1.9, 2.1)

                person1_Pub.publish(self.person1)
                self.person1.pose.position.x = 2 # random.uniform(2.0, 3.0)
                self.person1.pose.position.y = 2 # random.uniform(2.0, 3.0)
                self.person1.pose.position.z = 2 # random.uniform(1.5, 2.0)

            rate.sleep()

    def callback_UAV_2(self, data):
        #lets print some information
        print "obst_UAV2:\t", data.pose.position.x, ", ", data.pose.position.y, ", ", data.pose.position.z
        print

    def callback_UAV_3(self, data):
        #lets print some information
        print "obst_UAV3:\t", data.pose.position.x, ", ", data.pose.position.y, ", ", data.pose.position.z
        print

    def callback_UAV_4(self, data):
        #lets print some information
        print "obst_UAV4:\t", data.pose.position.x, ", ", data.pose.position.y, ", ", data.pose.position.z
        print

pub_obst()
