import rospy
# from std_msgs.msg import String
from visualization_msgs.msg import Marker


def initPathMarkers():

    # cannot write in one equation!!!
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
    neighbourPoint.scale.x  = neighbourPoint.scale.y    = neighbourPoint.scale.z    = 0.3
    finalPath.scale.x       = 0.5 # scale.x controls the width of the line segments

    sourcePoint.color.g     = 1.0
    goalPoint.color.r       = 1.0
    neighbourPoint.color.r  = 0.8
    neighbourPoint.color.g  = 0.4

    finalPath.color.r = 0.2
    finalPath.color.g = 0.2
    finalPath.color.b = 1.0

    sourcePoint.color.a     = 1.0
    goalPoint.color.a       = 1.0
    neighbourPoint.color.a  = 1.0
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
    boundary.scale.x    = 1 # scale.x controls the width of the line segments
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
    obstacle.type = 8 # Points

    obstacle.pose.orientation.w = 1.0
    obstacle.pose.position.x = 0.0
    obstacle.pose.position.y = 0.0
    obstacle.pose.position.z = 0.0

    obstacle.scale.x = obstacle.scale.y = obstacle.scale.z = 0.5

    obstacle.color.r = 0.0
    obstacle.color.g = 0.0
    obstacle.color.b = 0.0
    obstacle.color.a = 1.0

    return obstacle


def initPublishers():
    pathPub = rospy.Publisher('pp_markers', Marker, queue_size=10) # rostopic name
    return pathPub
