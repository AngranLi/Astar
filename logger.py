

# If I use rosbags

# rosbag record -o bag_test /crazyflie/battery /crazyflie/imu /UAV_1/pose /path_planner_rrt

# I want to print:

#############################################################################
#
#	Header:
#		Start location
#		End Location
#		Start time
#
#############################################################################
#
#
#	Path info?
#
#
#
#############################################################################
#
#
#	#time	#position	#goal	#imu	#battery voltage
#
#
#############################################################################

# example print info
#self.log_file.write(time.strftime("%H%M%S") + "\t" + str(self.pos.x) + ","+ str(self.pos.y) + ","+ str(self.pos.z) + "\t")

#Have a listener for:
#	UAV location
#	goal location
#	imu
#	battery info

#Every #n frames
#save to file

# if at location stop?
# if printed another path start a new one?

# battery topic:    /crazyflie/battery  #type: std_msgs/Float32
# imu topic:        /crazyflie/imu      #type: sensor_msgs/Imu
# pos topic:        /UAV_1/pose         #type: geometry_msgs/PoseStamped
# path topic        /path_planner_rrt   #type: visualization_msgs/Marker

import rospy
import time
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Point, Quaternion, Twist, PoseStamped
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Imu

class logger:
    #global variable - and default values
    def __init__(self):
        self.battery = 0
        self.angular_acc_x = 0
        self.angular_acc_y = 0
        self.angular_acc_z = 0
        self.linear_acc_x = 0
        self.linear_acc_y = 0
        self.linear_acc_z = 0
        self.position_x = 0
        self.position_y = 0
        self.position_z = 0

        self.pointlist = []

        self.goal_x = 0
        self.goal_y = 0
        self.goal_z = 0

    def callback(self, data):
        print 'X: ', data.pose.position.x

    def battery_listener(self, data):
        self.battery = data.data

    def imu_listener(self, data):
        self.angular_acc_x = data.angular_velocity.x
        self.angular_acc_y = data.angular_velocity.y
        self.angular_acc_z = data.angular_velocity.z
        self.linear_acc_x = data.linear_acceleration.x
        self.linear_acc_y = data.linear_acceleration.y
        self.linear_acc_z = data.linear_acceleration.z

    def position_listener(self, data):
        self.position_x = data.pose.position.x
        self.position_y = data.pose.position.y
        self.position_z = data.pose.position.z

    def goal_listener(self, data):
        self.goal_x = data.pose.position.x
        self.goal_y = data.pose.position.y
        self.goal_z = data.pose.position.z

    def path_listener(self, data):
        self.pointlist = []
        for i in data.points:
            self.pointlist.append(kVec(i.x * scale_path, i.y * scale_path, i.z * scale_path))

    def setup_logfile(self, fileWriter):
        #Header:
        #		Start location
        #		End Location
        #		Start time
        print 'setup_logfile'
        fileWriter.write("# Flight Log \n")
        fileWriter.write(time.strftime("# %d %m %y - %H:%M:%S\n"))
        # time  position    goal    linear acceleration     angular acceleration    battery level
        fileWriter.write("Time\t\tX,Y,Z\t\t\t\tgoalX,goalY,goalZ\t\tddx,ddy,ddz\t\t\tangularx,angulary,angularz\tbattery\n")

    def record_data(self, fileWriter):
        fileWriter.write(time.strftime("%H:%M:%S\t"))
        fileWriter.write("%f,%f,%f\t" % (self.position_x, self.position_y, self.position_z))
        fileWriter.write("%f,%f,%f\t" % (self.goal_x, self.goal_y, self.goal_z))
        fileWriter.write("%f,%f,%f\t" % (self.linear_acc_x, self.linear_acc_y, self.linear_acc_z))
        fileWriter.write("%f,%f,%f\t" % (self.position_x, self.position_y, self.position_z))
        fileWriter.write(str(self.battery))
        fileWriter.write("\n")
        #print self.battery

# check for input args...
if __name__ == '__main__':
    experimentTime = time.strftime('%m-%d %H:%M')

    # initialize ROS
    log_rate = 10
    rospy.init_node('myListener', log_level=rospy.DEBUG)
    rate = rospy.Rate(log_rate) #let take 10 measurements per minuite

    targetNum	= 4
    logDict		= {}
    fileWriter 	= {}
    batterySubDict	= {}
    imuSubDict		= {}
    positionSubDict	= {}
    goalSubDict		= {}
    for i in range(targetNum):
    	crazyflie_name = 'crazyflie'+str(i+1)
    	uav_name = 'UAV_'+str(i+1)
    	logDict[i] = logger()
    	# setup variabls:
    	fileWriter[i] = open(experimentTime+' '+uav_name+'.log', 'w+')
    	logDict[i].setup_logfile(fileWriter[i])

    	# set up listeners:
    	batterySubDict[i] 	= rospy.Subscriber(crazyflie_name + "/battery", Float32, logDict[i].battery_listener)
    	imuSubDict[i]		= rospy.Subscriber(crazyflie_name + "/imu", Imu, logDict[i].imu_listener)
    	positionSubDict[i]	= rospy.Subscriber(uav_name + "/pose", PoseStamped, logDict[i].position_listener)
    	#path_sub = rospy.Subscriber("path_planner_rrt", Marker, log.path_listener)
    	goalSubDict[i]		= rospy.Subscriber(uav_name + "/cmd_goal", PoseStamped, logDict[i].goal_listener)
    	
    # crazyflie_name = "crazyflie1"
    # uav_name = "UAV_1"
    # log = logger()
    # #setup variabls:
    # fileWriter = open(fileName, "w+")
    # log.setup_logfile(fileWriter)
    # #init ros
    # log_rate = 10
    # rospy.init_node('myListener', log_level=rospy.DEBUG)
    # rate = rospy.Rate(log_rate) #let take 10 measurements per minuite

    # #set up listeners
    # battery_sub = rospy.Subscriber(crazyflie_name + "/battery", Float32, log.battery_listener)
    # imu_sub = rospy.Subscriber(crazyflie_name + "/imu", Imu, log.imu_listener)
    # position_sub = rospy.Subscriber(uav_name + "/pose", PoseStamped, log.position_listener)
    # #path_sub = rospy.Subscriber("path_planner_rrt", Marker, log.path_listener)
    # goal_sub = rospy.Subscriber(uav_name + "/cmd_goal", PoseStamped, log.goal_listener)

    while not rospy.is_shutdown():
        #do loop here...
        for i in range(targetNum):
        	logDict[i].record_data(fileWriter[i])
        rate.sleep()

    #close the file etc...
    for i in range(targetNum):
    	fileWriter[i].close()
    print 'ended'