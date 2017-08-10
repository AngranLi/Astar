import random
import rospy
import inc
import initEnv
import prioQ
import init
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker, MarkerArray

rospy.init_node('dstar_node', anonymous=True) # rosnode name
(pathPub, pointPub, boundPub, obstPub) = init.initPublishers()

point1 = init.initPointMarkers()
point2 = init.initPointMarkers()
point1.color.g = 1
point2.color.r = 1

tempPoint = Point()
tempPoint.x = 15
tempPoint.y = 15
point1.points.append(tempPoint)
point1.pose.position.x = 15
point1.pose.position.y = 15

tempPoint = Point()
tempPoint.x = 20
tempPoint.y = 20
point2.points.append(tempPoint)
point2.pose.position.x = 20
point2.pose.position.y = 20

for i in range(5):
	print 'publishing point1...'
	pointPub.publish(point1)
	rospy.sleep(5)
	print 'publishing point2...'
	pointPub.publish(point2)
	rospy.sleep(5)

