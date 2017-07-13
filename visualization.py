import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped, Point
import initialization

def setPathMarkers(finalTrajectory, came_from):
    (sourcePoint, goalPoint, neighbourPoint, finalPath) = initialization.initPathMarkers()

    # neighbourPoint
    searchedPoints = []
    for key in came_from:
        searchedPoints.append(came_from[key])
    searchedPoints.remove(None)

    for i in range(len(searchedPoints)):
        tempPoint = Point()
        tempPoint.z = 0
        tempPoint.x = searchedPoints[i][0]
        tempPoint.y = searchedPoints[i][1]
        neighbourPoint.points.append(tempPoint)

    # finalPath
    for i in range(len(finalTrajectory)):
        tempPoint = Point()
        tempPoint.z = 0
        tempPoint.x = finalTrajectory[i][0]
        tempPoint.y = finalTrajectory[i][1]
        finalPath.points.append(tempPoint)

    # sourcePoint
    tempPoint = Point()
    tempPoint.x = finalTrajectory[0][0]
    tempPoint.y = finalTrajectory[0][1]
    tempPoint.z = 0
    sourcePoint.points.append(tempPoint)
    sourcePoint.pose.position.x = finalTrajectory[0][0]
    sourcePoint.pose.position.y = finalTrajectory[0][1]

    # goalPoint
    tempPoint = Point()
    tempPoint.x = finalTrajectory[len(finalTrajectory)-1][0]
    tempPoint.y = finalTrajectory[len(finalTrajectory)-1][1]
    tempPoint.z = 0
    goalPoint.points.append(tempPoint)
    goalPoint.pose.position.x = finalTrajectory[len(finalTrajectory)-1][0]
    goalPoint.pose.position.y = finalTrajectory[len(finalTrajectory)-1][1]

    return (sourcePoint, goalPoint, neighbourPoint, finalPath)


def setBoundary(length_of_map, width_of_map):
    boundary= initialization.initBoundMarker()

    # first point
    tempPoint = Point()
    tempPoint.x = 0
    tempPoint.y = 0
    tempPoint.z = 0
    boundary.points.append(tempPoint)

    # second point
    tempPoint = Point()
    tempPoint.x = length_of_map
    tempPoint.y = 0
    tempPoint.z = 0
    boundary.points.append(tempPoint)

    # third point
    tempPoint = Point()
    tempPoint.x = length_of_map
    tempPoint.y = width_of_map
    tempPoint.z = 0
    boundary.points.append(tempPoint)

    # fourth point
    tempPoint = Point()
    tempPoint.x = 0
    tempPoint.y = width_of_map
    tempPoint.z = 0
    boundary.points.append(tempPoint)

    # first point again to complete the box
    tempPoint = Point()
    tempPoint.x = 0
    tempPoint.y = 0
    tempPoint.z = 0
    boundary.points.append(tempPoint)

    return boundary


def setObstacle(walls):
    obstacle = initialization.initObstMarker()
    for point in walls:
        tempPoint = Point()
        tempPoint.x = point[0]
        tempPoint.y = point[1]
        tempPoint.z = 0
        obstacle.points.append(tempPoint)
    return obstacle
