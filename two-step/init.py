import rospy
# from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


def initPathMarkers():
    # cannot written in one equation!!!
    sourcePoint     = Marker()
    goalPoint       = Marker()
    neighbourPoint  = Marker()
    finalPath       = Marker()

    sourcePoint.header.frame_id     = 'path_planner'
    goalPoint.header.frame_id       = 'path_planner'
    neighbourPoint.header.frame_id  = 'path_planner'
    finalPath.header.frame_id       = 'path_planner'

    sourcePoint.header.stamp    = rospy.get_rostime()
    goalPoint.header.stamp      = rospy.get_rostime()
    neighbourPoint.header.stamp = rospy.get_rostime()
    finalPath.header.stamp      = rospy.get_rostime()

    sourcePoint.ns      = "path_planner"
    goalPoint.ns        = "path_planner"
    neighbourPoint.ns   = "path_planner"
    finalPath.ns        = "path_planner"

    sourcePoint.action      = 0     # add/modify an object
    goalPoint.action        = 0
    neighbourPoint.action   = 0
    finalPath.action        = 0

    sourcePoint.id      = 0
    goalPoint.id        = 1
    neighbourPoint.id   = 2
    finalPath.id        = 3

    # sourcePoint.text      = 'sourcePoint'
    # goalPoint.text        = 'goalPoint'
    # neighbourPoint.text   = 'neighbourPoint'
    # finalPath.text        = 'finalPath'

    sourcePoint.type    = 2 # Sphere
    goalPoint.type      = 2
    neighbourPoint.type = 8 # Points
    finalPath.type      = 4 # Line Strip

    sourcePoint.pose.orientation.w      = 1.0
    goalPoint.pose.orientation.w        = 1.0
    neighbourPoint.pose.orientation.w   = 1.0
    finalPath.pose.orientation.w        = 1.0

    sourcePoint.pose.position.x = 0.0
    sourcePoint.pose.position.y = 0.0
    sourcePoint.pose.position.z = 0.0

    goalPoint.pose.position.x = 10.0
    goalPoint.pose.position.y = 10.0
    goalPoint.pose.position.z = 0.0

    neighbourPoint.pose.position.x = 0.0
    neighbourPoint.pose.position.y = 0.0
    neighbourPoint.pose.position.z = 0.0

    sourcePoint.scale.x     = sourcePoint.scale.y       = sourcePoint.scale.z       = 1.0
    goalPoint.scale.x       = goalPoint.scale.y         = goalPoint.scale.z         = 1.0
    neighbourPoint.scale.x  = neighbourPoint.scale.y    = neighbourPoint.scale.z    = 0.1
    finalPath.scale.x       = 0.5 # scale.x controls the width of the line segments

    sourcePoint.color.g     = 1.0
    sourcePoint.color.b     = 1.0
    goalPoint.color.r       = 1.0
    goalPoint.color.b       = 1.0
    neighbourPoint.color.r  = 0.8
    neighbourPoint.color.g  = 0.4

    finalPath.color.r = 0.2
    finalPath.color.g = 0.2
    finalPath.color.b = 1.0

    sourcePoint.color.a     = 1.0
    goalPoint.color.a       = 1.0
    neighbourPoint.color.a  = 0.5
    finalPath.color.a       = 1.0

    return (sourcePoint, goalPoint, neighbourPoint, finalPath)

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


def initObstMarker():
    obstacle = Marker()

    obstacle.header.frame_id = 'path_planner'
    obstacle.header.stamp = rospy.get_rostime()
    obstacle.ns = "path_planner"
    obstacle.action = 0 # add/modify an object
    obstacle.id = 111
    obstacle.type = 3 # Cylinder

    obstacle.pose.orientation.w = 1.0
    obstacle.pose.position.x = 0.0
    obstacle.pose.position.y = 0.0
    obstacle.pose.position.z = 0.0

    obstacle.scale.x = obstacle.scale.y = obstacle.scale.z = 0

    obstacle.color.r = 1.0
    obstacle.color.g = 1.0
    obstacle.color.b = 1.0
    obstacle.color.a = 0.6

    return obstacle


def initPublishers():
    pathPub     = rospy.Publisher('path_planner_rrt', Marker, queue_size=10) # rostopic name
    pointsPub   = rospy.Publisher('points_markers', Marker, queue_size=10)
    boundPub    = rospy.Publisher('bound_markers', Marker, queue_size=10)
    obstPub     = rospy.Publisher('obstMarkers_array', MarkerArray, queue_size=10)
    roughPub    = rospy.Publisher('roughPath_markers', Marker, queue_size=10)
    return (pathPub, pointsPub, boundPub, obstPub, roughPub)

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
