import rospy
import collections
import heapq
import random
import math
import timeit
import string
import init
import visualization
from geometry_msgs.msg import PointStamped, Point, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray


class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1] # return the item (point)


class SquareGrid(object):
    def __init__(self, mapBound_grid):
        self.length = mapBound_grid[0]
        self.width  = mapBound_grid[1]
        self.height = mapBound_grid[2]
        # self.walls = [] # !!! Stop using diagram.walls to represent obstacles

    def in_bounds(self, point):
        (x, y, z) = point
        return 0 <= x <= self.length and 0 <= y <= self.width and 0<= z <= self.height

    def passable(self, point, obstDict):
        for key in obstDict:
            if obstDict[key].conflict(point):
                return False
        else:
            return True
        # return point not in self.walls

    def neighbors(self, point, obstDict):
        (x, y, z) = point
        # results = [(x+1, y), (x, y-1), (x-1, y), (x, y+1)]
        results = [(x+1, y, z), (x+1, y, z+1), (x, y, z+1), (x+1, y+1, z), (x+1, y+1, z+1),
                    (x, y+1, z), (x, y+1, z+1), (x-1, y+1, z), (x-1, y+1, z+1),
                    (x-1, y, z), (x-1, y, z+1), (x-1, y-1, z), (x-1, y-1, z+1),
                    (x, y-1, z), (x, y-1, z+1), (x+1, y-1, z), (x+1, y-1, z+1),
                    (x, y, z-1), (x+1, y, z-1), (x+1, y+1, z-1), (x, y+1, z-1), (x-1, y+1, z-1),
                    (x-1, y, z-1), (x-1, y-1, z-1), (x, y-1, z-1), (x+1, y-1, z-1)]
        # if (x + y) % 2 == 0: results.reverse() # aesthetics [Attention!]
        results = filter(self.in_bounds, results)
        ''' Different passability for rough and refined map '''
        results = filter(lambda x: self.passable(x, obstDict), results)
        return results


class GridWithWeights(SquareGrid):
    def __init__(self, mapBound_grid):
        super(GridWithWeights, self).__init__(mapBound_grid)
        self.weights = {}

    def cost(self, from_node, to_node):
        return self.weights.get(to_node, 1) # default is 1


class Obstacle():
    def __init__(self, centre_point, radius, height, mapHeight_grid, text):
        self.centre_point = centre_point
        self.radius = radius
        self.radiussq = radius**2
        self.height = height
        self.text = text

        if self.text == 'obst_UAV':
            self.top    = min(mapHeight_grid, self.centre_point[2] + self.height/2)
            self.bottom = max(0, self.centre_point[2] - self.height/2)
        elif self.text == 'obst_UGV':
            self.top    = min(mapHeight_grid, self.centre_point[2])
            self.bottom = max(0, self.centre_point[2] - self.height)
        elif self.text == 'obst_person':
            self.top    = mapHeight_grid
            self.bottom = 0

    def movePos(self, centre_point, mapHeight_grid):
        self.centre_point = centre_point
        if self.text == 'obst_UAV':
            self.top    = min(mapHeight_grid, self.centre_point[2] + self.height/2)
            self.bottom = max(0, self.centre_point[2] - self.height/2)
        elif self.text == 'obst_UGV':
            self.top    = min(mapHeight_grid, self.centre_point[2])
            self.bottom = max(0, self.centre_point[2] - self.height)
        elif self.text == 'obst_person':
            self.top    = mapHeight_grid
            self.bottom = 0

    def conflict(self, point):
        # if in the vertical range
        if (point[0]-self.centre_point[0])**2 + (point[1]-self.centre_point[1])**2 > self.radiussq:
            return False
        elif self.bottom <= point[2] <= self.top:
            return True
        else:
            return False


def checkifPathBlocked(obstDict, trajectory):
    for key in obstDict:
        for point in trajectory:
            if obstDict[key].conflict(point):
                return True
    else:
        return False


def reconstruct_path(came_from, start, goal):
    current = goal
    path = [current]
    while current != start:
        if current in came_from:
            current = came_from[current]
            path.append(current)
        else:
            rospy.logfatal('No available path!')
            print 'goalpoint: ', goal
            print
            print 'currentpoint: ', start
            # print
            # print 'walls: \n', diagram.walls
            f = open('came_from', 'w')
            for key in came_from:
                f.write(str(key) + ':' + str(came_from[key]) + '  ')
            f.close()
            path.reverse()
            return path
    # path.append(start) # optional
    path.reverse() # optional
    return path


def heuristic(a, b, heuristic_coeff):
    (x1, y1, z1) = a
    (x2, y2, z2) = b
    # return max(abs(x1-x2), abs(y1-y2), abs(z1-z2))
    return math.sqrt((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2) * heuristic_coeff


def a_star_search(graph, obstDict, start, goal, heuristic_coeff):
    temp_obstDict = obstDict.copy()
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    global heap_percolation

    while not frontier.empty():
        current = frontier.get()
        # rospy.logfatal(current)
        if current == goal:
            break
        # rospy.logfatal(goal)

        for next in graph.neighbors(current, obstDict.copy()):
            # if the neighbourPoint and the current are on a diagonal of cubic
            if abs(next[0]-current[0]) + abs(next[1]-current[1]) + abs(next[2]-current[2]) == 3:
                new_cost = cost_so_far[current] + math.sqrt(3)*graph.cost(current, next)
            # if the neighbourPoint and the current are on a diagonal of square
            elif abs(next[0]-current[0]) + abs(next[1]-current[1]) + abs(next[2]-current[2]) == 2:
                new_cost = cost_so_far[current] + math.sqrt(2)*graph.cost(current, next)
            else:
                new_cost = cost_so_far[current] + graph.cost(current, next)
            # Percolate the priority queue
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next, heuristic_coeff)
                frontier.put(next, priority) # push and reoder the queue
                came_from[next] = current
                heap_percolation += 1

    return came_from, cost_so_far


def callback_obst_UAV1(centre_point):
    # rospy.logwarn(len(diagram.walls))
    # rospy.logwarn((centre_point.pose.position.x, centre_point.pose.position.y,
    #                 centre_point.pose.position.z))
    global obstDict_rough
    global obstDict_fine
    global pathBlocked
    global roughTrajectory

    obstacle_rough = Obstacle(init.gridalize((centre_point.pose.position.x,
                centre_point.pose.position.y, centre_point.pose.position.z), scale_rough),
                init.gridalize(0.25 *1.2, scale_rough), init.gridalize(2, scale_rough), mapBound_grid_rough[2], 'obst_UAV')

    obstacle_fine = Obstacle(init.gridalize((centre_point.pose.position.x,
                centre_point.pose.position.y, centre_point.pose.position.z), scale_fine),
                init.gridalize(0.25, scale_fine), init.gridalize(2, scale_fine), mapBound_grid_fine[2], 'obst_UAV')

    obstDict_rough['obst_UAV1']  = obstacle_rough
    obstDict_fine['obst_UAV1']   = obstacle_fine

    pathBlocked = checkifPathBlocked(obstDict_rough.copy(), roughTrajectory)

    callback_obst_UAV1_flg = True


def callback_obst_UGV1(centre_point):
    # rospy.logwarn(len(diagram.walls))
    # rospy.logwarn((centre_point.pose.position.x, centre_point.pose.position.y,
    #                 centre_point.pose.position.z))
    global obstDict_rough
    global obstDict_fine
    global pathBlocked
    global roughTrajectory

    obstacle_rough = Obstacle(init.gridalize((centre_point.pose.position.x,
                centre_point.pose.position.y, centre_point.pose.position.z), scale_rough),
                init.gridalize(0.5 *1.2, scale_rough), init.gridalize(2, scale_rough), mapBound_grid_rough[2], 'obst_UGV')

    obstacle_fine = Obstacle(init.gridalize((centre_point.pose.position.x,
                centre_point.pose.position.y, centre_point.pose.position.z), scale_fine),
                init.gridalize(0.5, scale_fine), init.gridalize(2, scale_fine), mapBound_grid_fine[2], 'obst_UGV')

    obstDict_rough['obst_UGV1']  = obstacle_rough
    obstDict_fine['obst_UGV1']   = obstacle_fine

    pathBlocked = checkifPathBlocked(obstDict_rough, roughTrajectory)

    callback_obst_UGV1_flg = True


def callback_obst_person1(centre_point):
    # rospy.logwarn(len(diagram.walls))
    # rospy.logwarn((centre_point.pose.position.x, centre_point.pose.position.y,
    #                 centre_point.pose.position.z))
    global obstDict_rough
    global obstDict_fine
    global pathBlocked
    global roughTrajectory

    obstacle_rough = Obstacle(init.gridalize((centre_point.pose.position.x,
                centre_point.pose.position.y, centre_point.pose.position.z), scale_rough),
                init.gridalize(1 *1.2, scale_rough), init.gridalize(3, scale_rough), mapBound_grid_rough[2], 'obst_person')

    obstacle_fine = Obstacle(init.gridalize((centre_point.pose.position.x,
                centre_point.pose.position.y, centre_point.pose.position.z), scale_fine),
                init.gridalize(1, scale_fine), init.gridalize(3, scale_fine), mapBound_grid_fine[2], 'obst_person')

    obstDict_rough['obst_person1']  = obstacle_rough
    obstDict_fine['obst_person1']   = obstacle_fine

    pathBlocked = checkifPathBlocked(obstDict_rough, roughTrajectory)

    callback_obst_person1_flg = True


def callback_current(data):  # data contains the current point
    # rospy.loginfo((data.pose.position.x, data.pose.position.y, data.pose.position.z))
    global current_position
    current_position = (data.pose.position.x, data.pose.position.y, data.pose.position.z)
    callback_current_flg = True


def callback_target(data):  # data contains the target point
    # rospy.loginfo((data.points[1].x, data.points[1].y, data.points[1].z))
    global target_position
    target_position = (data.pose.position.x, data.pose.position.y, data.pose.position.z)
    callback_target_flg = True

def generateRoughTrajectory():
    print
    print '====================================='
    print 'Generating the rough path...'

    global mapBound_grid_rough
    global diagram_rough
    global current_position
    global target_position
    global scale_rough
    global obstDict_rough
    global heap_percolation

    # initialize environment
    startPoint  = tuple(init.gridalize(current_position, scale_rough))
    endPoint    = tuple(init.gridalize(target_position, scale_rough))

    heap_percolation = 0

    print 'startPoint: ', startPoint
    print 'endPoint: ', endPoint

    for key in obstDict_rough:
        if obstDict_rough[key].conflict(startPoint):
            # set the startPoint to the nearest point outside the obstacle
            dist = math.sqrt((startPoint[0]-obstDict_rough[key].centre_point[0])**2 + (startPoint[1]-obstDict_rough[key].centre_point[1])**2)
            startPoint = tuple((int((startPoint[0]-obstDict_rough[key].centre_point[0]) * (obstDict_rough[key].radius/dist) + obstDict_rough[key].centre_point[0]+1),
                            int((startPoint[1]-obstDict_rough[key].centre_point[1]) * (obstDict_rough[key].radius/dist) + obstDict_rough[key].centre_point[1]+1), startPoint[2]))
            print 'changd startPoint: ', startPoint
            break

        if obstDict_rough[key].conflict(endPoint):
            rospy.logfatal('Destination conflicts with obstacle!')
            print 'Centre of obstacle: ', obstDict_rough[key].centre_point
            print 'Radius of obstacle: ', obstDict_rough[key].radius
            rospy.signal_shutdown()

    # Plan the path
    start_time = timeit.default_timer()
    came_from, cost_so_far = a_star_search(diagram_rough, obstDict_rough, startPoint, endPoint, 1.01)
    roughTrajectory = reconstruct_path(came_from, start=startPoint, goal=endPoint)
    execution_time = timeit.default_timer() - start_time

    # searchedPoints = []
    # for key in came_from:
    #     searchedPoints.append(came_from[key])
    # searchedPoints.remove(None)
    # Performance measurement
    len_of_path = cost_so_far[endPoint]
    # vertex_expension = len(searchedPoints)
    vertex_expension = len(came_from)

    print
    print 'Length of path: ', float(len_of_path / scale_rough)
    print 'Path planning execution time: ', execution_time
    print 'Vetices expanded: ', vertex_expension
    print 'Heap percolated: ', heap_percolation

    return roughTrajectory


def generateRefinedTrajectory(roughTrajectory):
    print '-------------------------------------'
    print 'Generating the refined path...'

    global mapBound_grid_fine
    global diagram_fine
    global current_position
    global scale_rough
    global scale_fine
    global obstDict_fine
    global heap_percolation
    global scaleRatio

    # initialize environment
    startPoint = tuple(init.gridalize(current_position, scale_fine))
    # (roughTrajectory[i-2][0]*scaleRatio, roughTrajectory[i-2][1]*scaleRatio, roughTrajectory[i-2][2]*scaleRatio)
    distsq_min = 10000000000
    closestPoint_id = 0
    for i in range(len(roughTrajectory)):
        distsq = (roughTrajectory[i][0]*scaleRatio -startPoint[0])**2 + (roughTrajectory[i][1]*scaleRatio -startPoint[1])**2 + (roughTrajectory[i][2]*scaleRatio -startPoint[2])**2
        if distsq < distsq_min:
            distsq_min = distsq
            closestPoint_id = i

    print
    print 'startPoint: ', startPoint
    print 'closestPoint_id: ', closestPoint_id
    print 'closestPoint: ', (roughTrajectory[closestPoint_id][0] *scaleRatio,
                roughTrajectory[closestPoint_id][1] *scaleRatio, roughTrajectory[closestPoint_id][2] *scaleRatio)
    try:
        endPoint = (roughTrajectory[closestPoint_id+2][0] *scaleRatio, roughTrajectory[closestPoint_id+2][1] *scaleRatio,
                    roughTrajectory[closestPoint_id+2][2] *scaleRatio)
    except IndexError:
        endPoint = (roughTrajectory[-1][0] *scaleRatio, roughTrajectory[-1][1] *scaleRatio,
                    roughTrajectory[-1][2] *scaleRatio)

    heap_percolation = 0
    print 'startPoint: ', startPoint
    print 'endPoint: ', endPoint

    boundMarker     = visualization.setBoundary(mapBound_grid_fine)
    obstMarkerArray = visualization.setObstacle(obstDict_fine.values())

    # Publish the boundary and obstacle
    boundPub.publish(boundMarker)
    obstPub.publish(obstMarkerArray)

    for key in obstDict_fine:
        if obstDict_fine[key].conflict(startPoint):
            # set the startPoint to the nearest point outside the obstacle
            try:
                dist = math.sqrt((startPoint[0]-obstDict_fine[key].centre_point[0])**2 + (startPoint[1]-obstDict_fine[key].centre_point[1])**2)
                startPoint = tuple((int((startPoint[0]-obstDict_fine[key].centre_point[0]) * (obstDict_fine[key].radius/dist) + obstDict_fine[key].centre_point[0]+1),
                                int((startPoint[1]-obstDict_fine[key].centre_point[1]) * (obstDict_fine[key].radius/dist) + obstDict_fine[key].centre_point[1]+1), startPoint[2]))
                print 'changd startPoint: ', startPoint
                break
            except ZeroDivisionError:
                print 'startPoint x/y is ', startPoint[0], startPoint[1]
                print 'obst centre point is ', obstDict_fine[key].centre_point[0], obstDict_fine[key].centre_point[1]
                print 'obst radius is ', obstDict_fine[key].radius
                print 'dist is ', dist
                rospy.signal_shutdown()

        if obstDict_fine[key].conflict(endPoint):
            rospy.logfatal('Destination conflicts with obstacle!')
            print 'Centre of obstacle: ', obstDict_fine[key].centre_point
            print 'Radius of obstacle: ', obstDict_fine[key].radius
            rospy.signal_shutdown()

    # Plan the path
    print
    print 'Planning path...'
    start_time = timeit.default_timer()
    came_from, cost_so_far = a_star_search(diagram_fine, obstDict_fine, startPoint, endPoint, 1.5)
    finalTrajectory = reconstruct_path(came_from, start=startPoint, goal=endPoint)
    execution_time = timeit.default_timer() - start_time

    # These four values are all visualization markers!
    (sourcePoint, goalPoint, neighbourPoint,
        finalPath) = visualization.setPathMarkers(finalTrajectory, came_from)
    finalPath.text = '0'

    # Publish the path
    pointsPub.publish(sourcePoint)
    pointsPub.publish(goalPoint)
    pointsPub.publish(neighbourPoint)
    pathPub.publish(finalPath)

    print
    print 'Path sent.'

    # Performance measurement
    len_of_path = cost_so_far[endPoint]
    vertex_expension = len(neighbourPoint.points)
    # f = open('cost_so_far', 'w')
    # f.write(str(cost_so_far))
    # f.close()
    print
    print 'Length of path: ', float(len_of_path/scale_fine)
    print 'Path planning execution time: ', execution_time
    print 'Vetices expanded: ', vertex_expension
    print 'Heap percolated: ', heap_percolation

    return finalTrajectory


###############################################################################
###############################################################################
###############################################################################
''' Set original environment '''
mapBound_metre = (5, 5, 3)      # 3D boundary of the operating environment
scale_rough = 4
scale_fine = 50
scaleRatio = float(scale_fine/scale_rough)
refineRatio = int((scale_fine/scale_rough)**(1.0/3))

startPoint  = (0, 0, 0)
endPoint    = (0, 0, 0)
current_position  = (1, 1, 0.5)
target_position   = (4.5, 4, 2.5)

''' Initialize ROS node and publishers '''
rospy.init_node('astar_node', anonymous=True) # rosnode name
(pathPub, pointsPub, boundPub, obstPub, roughPub) = init.initPublishers() # initialize publishers
rospy.sleep(0.3) # it takes time to initialize publishers
rate = rospy.Rate(10) # loop runs at x Hertz

''' Set original value of flags '''
gotPath = False
pathBlocked = True
callback_obst_UAV1_flg = True
callback_obst_UGV1_flg = True
callback_obst_person1_flg = True
callback_current_flg = True
callback_target_flg = True

''' Performance measurement '''
len_of_path = 0
execution_time = 0
vertex_expension = 0
heap_percolation = 0

''' Initialize grid map '''
# rough grid map
mapBound_grid_rough = init.gridalize(mapBound_metre, scale_rough)
diagram_rough = GridWithWeights(mapBound_grid_rough)
# fine grid map
mapBound_grid_fine = init.gridalize(mapBound_metre, scale_fine)
diagram_fine = GridWithWeights(mapBound_grid_fine)

''' Initialize obstacle array '''
obstDict_rough = {}
obstDict_fine  = {}
roughTrajectory = []

''' Loop for path planning '''
while not rospy.is_shutdown():

    # receive obstacle postion
    while callback_obst_UAV1_flg:
        obstSub = rospy.Subscriber('/UAV_2/pose', PoseStamped, callback_obst_UAV1)
        callback_obst_UAV1_flg = False
    while callback_obst_UGV1_flg:
        obstSub = rospy.Subscriber('/UAV_3/pose', PoseStamped, callback_obst_UGV1)
        callback_obst_UGV1_flg = False
    while callback_obst_person1_flg:
        obstSub = rospy.Subscriber('/UAV_4/pose', PoseStamped, callback_obst_person1)
        callback_obst_person1_flg = False

    # receive current position of UAV
    while callback_current_flg:
        ppSub   = rospy.Subscriber('/UAV_1/pose', PoseStamped, callback_current)
        callback_current_flg = False
    # receive requested destination
    while callback_target_flg:
        ppSub   = rospy.Subscriber('target_request', PoseStamped, callback_target)
        callback_target_flg = False

    ''' Generate a rough trajectory'''
    if (not gotPath) or pathBlocked:
        roughTrajectory = generateRoughTrajectory() # plan a new rough path
        gotPath = True
        pathBlocked = False
        (orig_point, destination, roughPath) = visualization.setRoughMarkers(roughTrajectory, scaleRatio)
        roughPub.publish(orig_point)
        roughPub.publish(destination)
        roughPub.publish(roughPath)

    ''' Refine the trajectory '''
    finalTrajectory = generateRefinedTrajectory(roughTrajectory)

    # rospy.sleep(2.)
    rate.sleep()
        # rospy.signal_shutdown()
