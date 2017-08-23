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

    def setup_logfile(self, file_writer):
        #Header:
        #		Start location
        #		End Location
        #		Start time
        print 'setup_logfile'
        file_writer.write("# Flight Log\n")
        file_writer.write(time.strftime("# %d %m %y - %H:%M:%S\n"))
        # time  position    goal    linear acceleration     angular acceleration    battery level
        file_writer.write("Time\t\tX,Y,Z\t\t\t\tgoalX,goalY,goalZ\t\tddx,ddy,ddz\t\t\tangularx,angulary,angularz\tbattery\n")

    def record_data(self, file_writer):
        file_writer.write(time.strftime("%H:%M:%S\t"))
        file_writer.write("%f,%f,%f\t" % (self.position_x, self.position_y, self.position_z))
        file_writer.write("%f,%f,%f\t" % (self.goal_x, self.goal_y, self.goal_z))
        file_writer.write("%f,%f,%f\t" % (self.linear_acc_x, self.linear_acc_y, self.linear_acc_z))
        file_writer.write("%f,%f,%f\t" % (self.position_x, self.position_y, self.position_z))
        file_writer.write(str(self.battery))
        file_writer.write("\n")
        #print self.battery

tempDict = {}
for i in range(3):
	tempDict[i] = logger()

f = open('testfile.txt', 'w')
print type(f.name)
print f.name