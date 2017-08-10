import rospy
# from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

def initPathMarkers():
    # start/ goal/ current point
    path = Marker()

    path.header.frame_id   = 'path_planner'
    path.header.stamp      = rospy.get_rostime()
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

    return path

def initPointMarkers():
    # start/ goal/ current point
    point = Marker()

    point.header.frame_id   = 'path_planner'
    point.header.stamp      = rospy.get_rostime()
    point.ns        = "path_planner"
    point.action    = 0     # add/modify an object
    # point.text      = 'point'
    point.type = 2 # Sphere

    point.pose.orientation.w    = 1.0
    point.pose.position.x       = 0.0
    point.pose.position.y       = 0.0
    # point.pose.position.z     = 0.0
    point.scale.x = point.scale.y = point.scale.z = 1.0
    point.color.a = 1.0

    return point

def initBoundMarker():
    boundary = Marker()

    boundary.header.frame_id = 'path_planner'
    boundary.header.stamp = rospy.get_rostime()
    boundary.ns = "path_planner"
    boundary.action = 0     # add/modify an object
    boundary.id = 110
    boundary.type = 4 # Line Strip

    boundary.pose.orientation.w = 1.0
    boundary.scale.x = 1 # scale.x controls the width of the line segments

    boundary.color.r = 0.0
    boundary.color.g = 0.0
    boundary.color.b = 0.0
    boundary.color.a = 1.0

    return boundary


def initObstMarkers():
    obstacle = Marker()

    obstacle.header.frame_id  = 'path_planner'
    obstacle.header.stamp = rospy.get_rostime()
    obstacle.ns   = "path_planner"
    obstacle.action   = 0
    obstacle.id   = 30
    # obstacle.text   = 'obstacle'
    obstacle.type = 8 # Points

    obstacle.pose.orientation.w   = 1.0
    obstacle.pose.position.x = 0.0
    obstacle.pose.position.y = 0.0
    # obstacle.pose.position.z = 0.0

    obstacle.scale.x  = obstacle.scale.y    = obstacle.scale.z    = 0.5

    obstacle.color.r  = 0.0
    obstacle.color.g  = 0.0
    obstacle.color.b  = 0.0
    obstacle.color.a  = 1.0

    return obstacle


def initPublishers():
    pathPub     = rospy.Publisher('path_planner_rrt', Marker, queue_size=10) # rostopic name
    pointPub    = rospy.Publisher('points_markers', Marker, queue_size=10)
    boundPub    = rospy.Publisher('bound_markers', Marker, queue_size=10)
    obstPub     = rospy.Publisher('obstMarkers', Marker, queue_size=10)
    # roughPub    = rospy.Publisher('roughPath_markers', Marker, queue_size=10)
    return (pathPub, pointPub, boundPub, obstPub)

def gridalize(value, scale):
    if isinstance(value, tuple):
        result = []
        value = list(value)
        for i in range(len(value)):
            result.append(int(round(value[i] *scale)))
        return tuple(result)
    else:
        return int(round(value *scale))

def initRoughMarkers():
    orig_point  = Marker()
    destination = Marker()
    roughPath   = Marker()

    orig_point.header.frame_id  = 'path_planner'
    destination.header.frame_id = 'path_planner'
    roughPath.header.frame_id   = 'path_planner'

    orig_point.header.stamp     = rospy.get_rostime()
    destination.header.stamp    = rospy.get_rostime()
    roughPath.header.stamp      = rospy.get_rostime()

    orig_point.ns   = "path_planner"
    destination.ns  = "path_planner"
    roughPath.ns    = "path_planner"

    orig_point.action   = 0     # add/modify an object
    destination.action  = 0
    roughPath.action    = 0

    orig_point.id   = 20
    destination.id  = 21
    roughPath.id    = 22

    orig_point.type     = 2 # Sphere
    destination.type    = 2
    roughPath.type      = 4 # Line Strip

    orig_point.pose.orientation.w   = 1.0
    destination.pose.orientation.w  = 1.0
    roughPath.pose.orientation.w    = 1.0

    orig_point.pose.position.x = 0.0
    orig_point.pose.position.y = 0.0
    orig_point.pose.position.z = 0.0

    destination.pose.position.x = 10.0
    destination.pose.position.y = 10.0
    destination.pose.position.z = 0.0

    orig_point.scale.x  = orig_point.scale.y    = orig_point.scale.z    = 3.0
    destination.scale.x = destination.scale.y   = destination.scale.z   = 3.0
    roughPath.scale.x   = 0.5 # scale.x controls the width of the line segments

    orig_point.color.g  = 1.0
    destination.color.r = 1.0

    roughPath.color.g = 1.0
    roughPath.color.b = 1.0

    orig_point.color.a  = 1.0
    destination.color.a = 1.0
    roughPath.color.a   = 0.5

    return (orig_point, destination, roughPath)
