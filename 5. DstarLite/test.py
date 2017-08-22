import time
import timeit
import random
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped, Point, PoseStamped


# lst = []
# for i in range(5000):
#     lst.append((random.randint(0,500), random.randint(0,500)))


# f = open('l1', 'w')
# f.write(str(lst))
# f.close()
#
# start_time = timeit.default_timer()
# lst2 = list(set(lst))
# execution_time = timeit.default_timer() - start_time
#
# f = open('l2', 'w')
# f.write(str(lst2))
# f.close()
#
# print execution_time
#
# f = open('l2', 'r')
# temp_lst = list(f.read())
# print temp_lst
# print type(temp_lst)


# start/ goal/ current point
path = Marker()

path.header.frame_id   = 'path_planner'
path.ns     = "path_planner"
path.action = 0     # add/modify an object
path.id     = 4
# path.text      = 'path'
path.type = 4 # Line Strip

path.pose.orientation.w    = 1.0

path.scale.x = 0.5 # scale.x controls the width of the line segments

path.color.r = 0.2
path.color.g = 0.2
path.color.b = 1.0
path.color.a = 1.0


tempPoint = Point()
tempPoint.x = 7
tempPoint.y = 5
tempPoint.z = 8
path.points.append(tempPoint)

tempPoint = Point()
tempPoint.x = 7
tempPoint.y = 5
tempPoint.z = 8
path.points.append(tempPoint)

tempList = []
for i in range(len(path.points)):
    tempList.append((path.points[i].x, path.points[i].y, path.points[i].z))

f= open('test.txt', 'w')
f.write(str(tempList))
f.close()
