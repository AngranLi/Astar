import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped, Point
import init

def setPathMarkers(finalTrajectory, came_from):
    (sourcePoint, goalPoint, neighbourPoint, finalPath) = init.initPathMarkers()

    # neighbourPoint
    searchedPoints = []
    for key in came_from:
        searchedPoints.append(came_from[key])
    searchedPoints.remove(None)

    for i in range(len(searchedPoints)):
        tempPoint = Point()
        tempPoint.x = searchedPoints[i][0]
        tempPoint.y = searchedPoints[i][1]
        tempPoint.z = searchedPoints[i][2]
        neighbourPoint.points.append(tempPoint)

    # finalPath
    for i in range(len(finalTrajectory)):
        tempPoint = Point()
        tempPoint.x = finalTrajectory[i][0]
        tempPoint.y = finalTrajectory[i][1]
        tempPoint.z = finalTrajectory[i][2]
        finalPath.points.append(tempPoint)

    # sourcePoint
    tempPoint = Point()
    tempPoint.x = finalTrajectory[0][0]
    tempPoint.y = finalTrajectory[0][1]
    tempPoint.z = finalTrajectory[0][2]
    sourcePoint.points.append(tempPoint)
    sourcePoint.pose.position.x = finalTrajectory[0][0]
    sourcePoint.pose.position.y = finalTrajectory[0][1]
    sourcePoint.pose.position.z = finalTrajectory[0][2]

    # goalPoint
    tempPoint = Point()
    tempPoint.x = finalTrajectory[len(finalTrajectory)-1][0]
    tempPoint.y = finalTrajectory[len(finalTrajectory)-1][1]
    tempPoint.z = finalTrajectory[len(finalTrajectory)-1][2]
    goalPoint.points.append(tempPoint)
    goalPoint.pose.position.x = finalTrajectory[len(finalTrajectory)-1][0]
    goalPoint.pose.position.y = finalTrajectory[len(finalTrajectory)-1][1]
    goalPoint.pose.position.z = finalTrajectory[len(finalTrajectory)-1][2]

    return (sourcePoint, goalPoint, neighbourPoint, finalPath)


def setBoundary(length_of_map, width_of_map, height_of_map):
    boundary= init.initBoundMarker()

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

    # first point on the top
    tempPoint = Point()
    tempPoint.x = 0
    tempPoint.y = 0
    tempPoint.z = height_of_map
    boundary.points.append(tempPoint)

    # second point
    tempPoint = Point()
    tempPoint.x = length_of_map
    tempPoint.y = 0
    tempPoint.z = height_of_map
    boundary.points.append(tempPoint)

    # third point
    tempPoint = Point()
    tempPoint.x = length_of_map
    tempPoint.y = width_of_map
    tempPoint.z = height_of_map
    boundary.points.append(tempPoint)

    # fourth point
    tempPoint = Point()
    tempPoint.x = 0
    tempPoint.y = width_of_map
    tempPoint.z = height_of_map
    boundary.points.append(tempPoint)

    # first point again to complete the box
    tempPoint = Point()
    tempPoint.x = 0
    tempPoint.y = 0
    tempPoint.z = height_of_map
    boundary.points.append(tempPoint)

    return boundary


def setObstacle(obstArray):
    obstMarkerArray = MarkerArray()
    i = 0
    for item in obstArray:
        obstMarker = init.initObstMarker()
        obstMarker.scale.x = 2*item.radius
        obstMarker.scale.y = 2*item.radius
        obstMarker.scale.z = item.height
        obstMarker.pose.position.x = item.centre_point[0]
        obstMarker.pose.position.y = item.centre_point[1]
        if item.text == 'obst_UAV':
            obstMarker.pose.position.z = item.centre_point[2]
        elif item.text == 'obst_UGV':
            obstMarker.pose.position.z = item.centre_point[2] - item.height/2
        elif item.text == 'obst_person':
            obstMarker.pose.position.z = 30/2
        obstMarker.id = i
        i += 1
        obstMarkerArray.markers.append(obstMarker)
    return obstMarkerArray
