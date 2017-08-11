import random
import rospy
import inc
# import initEnv
# import prioQ
import init
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker, MarkerArray

permutation = []
for i in range(inc.DIRECTIONS):
    column = []
    for i in range(inc.DIRECTIONS):
        column.append(i)
    permutation.append(column)

print len(permutation),
print len(permutation[0])
print permutation