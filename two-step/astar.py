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

    def passable(self, point, obstArray):
        for i in range(len(obstArray)):
            if obstArray[i].conflict(point):
                return False
        else:
            return True
        # return point not in self.walls

    def neighbors(self, point, obstArray):
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
        results = filter(lambda x: self.passable(x, obstArray), results)
        return results


class GridWithWeights(SquareGrid):
    def __init__(self, mapBound_grid):
        super(GridWithWeights, self).__init__(mapBound_grid)
        self.weights = {}

    def cost(self, from_node, to_node):
        return self.weights.get(to_node, 1) # default is 1


class Obstacle():
    def __init__(self, centre_point, radius, height, text):
        self.centre_point = centre_point
        self.radius = radius
        self.radiussq = radius**2
        self.height = height
        self.text = text

    def setPos(self, centre_point):
        self.centre_point = centre_point

    def setRange(self, mapHeight_grid):
        if self.text == 'obst_UAV':
            self.top    = min(mapHeight_grid, self.centre_point[2] + self.height/2)
            self.bottom = max(0, self.centre_point[2] - self.height/2)
        elif self.text == 'obst_UGV':
            self.top    = min(mapHeight_grid, self.centre_point[2])
            self.bottom = max(0, self.centre_point[2] - self.height)
        elif self.text == 'obst_person':
            self.top    = mapHeight_grid
            self.bottom = 0

        # for visualization
        self.points = []
        # for i in range(self.left, self.right+1):
        #     for j in range(self.back, self.front+1):
        #         for k in range(self.bottom, self.top+1):
        #             self.points.append((i, j, k))

    def conflict(self, point):
        # if in the vertical range
        if (point[0]-self.centre_point[0])**2 + (point[1]-self.centre_point[1])**2 > self.radiussq:
            return False
        elif self.bottom <= point[2] <= self.top:
            return True
        else:
            return False


def ifPathBlocked(obstArray, trajectory):
    for item in obstArray:
        for point in trajectory:
            if item.conflict(point):
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


def heuristic(a, b):
    (x1, y1, z1) = a
    (x2, y2, z2) = b
    # return max(abs(x1-x2), abs(y1-y2), abs(z1-z2))
    return math.sqrt((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2) * 1.1


def a_star_search(graph, obstArray, start, goal):
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

        for next in graph.neighbors(current, obstArray):
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
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority) # push and reoder the queue
                came_from[next] = current
                heap_percolation += 1

    return came_from, cost_so_far

# need to
def callback_obst(centre_point):
    # rospy.logwarn(len(diagram.walls))
    # rospy.logwarn((centre_point.pose.position.x, centre_point.pose.position.y,
    #                 centre_point.pose.position.z))
    global obstArray_rough
    global obstArray_fine
    global pathBlocked
    global roughTrajectory

    obstacle_rough = Obstacle(init.gridalize((centre_point.pose.position.x,
                centre_point.pose.position.y, centre_point.pose.position.z), scale_rough),
                init.gridalize(0.5, scale_rough), init.gridalize(2, scale_rough), 'obst_UGV')
    obstacle_rough.setRange(mapBound_grid_rough[2])

    obstacle_fine = Obstacle(init.gridalize((centre_point.pose.position.x,
                centre_point.pose.position.y, centre_point.pose.position.z), scale_fine),
                init.gridalize(0.5, scale_fine), init.gridalize(2, scale_fine), 'obst_UGV')
    obstacle_fine.setRange(mapBound_grid_fine[2])

    obstArray_rough = [obstacle_rough]
    obstArray_fine  = [obstacle_fine]

    pathBlocked = ifPathBlocked(obstArray_rough, roughTrajectory)

    callback_obst_flg = True


def callback_current(data):  # data contains the current point
    # rospy.loginfo((data.pose.position.x, data.pose.position.y, data.pose.position.z))
    global current_point
    current_position = (int(data.pose.position.x * scale), int(data.pose.position.y * scale),
                        int(data.pose.position.z* scale))
    callback_current_flg = True


def callback_target(data):  # data contains the target point
    # rospy.loginfo((data.points[1].x, data.points[1].y, data.points[1].z))
    global target_point
    target_position = (int(data.pose.position.x * scale), int(data.pose.position.y * scale),
                        int(data.pose.position.z* scale))
    callback_target_flg = True

def generateRoughPath():
    global mapBound_grid_rough
    global diagram_rough
    global current_position
    global target_position
    global scale_rough
    global obstArray_rough
    global heap_percolation

    # initialize environment
    startPoint  = tuple(init.gridalize(current_position, scale_rough))
    endPoint    = tuple(init.gridalize(target_position, scale_rough))

    '''
    # initialize obstacles with position of their centre point
    obst_UAV1 = Obstacle(init.gridalize((0.5, 2.5, 1), scale_rough), init.gridalize(0.25, scale_rough),
                    init.gridalize(2, scale_rough), 'obst_UAV')
    obst_UAV2 = Obstacle(init.gridalize((1.5, 0.5, 1.5), scale_rough), init.gridalize(0.25, scale_rough),
                    init.gridalize(2, scale_rough), 'obst_UAV')
    obst_UAV3 = Obstacle(init.gridalize((3.5, 4.5, 2), scale_rough), init.gridalize(0.25, scale_rough),
                    init.gridalize(2, scale_rough), 'obst_UAV')
    obst_UGV1 = Obstacle(init.gridalize((1.5, 4.5, 2), scale_rough), init.gridalize(0.5, scale_rough),
                    init.gridalize(2, scale_rough), 'obst_UGV')
    obst_UGV2 = Obstacle(init.gridalize((4.5, 2.5, 2), scale_rough), init.gridalize(0.5, scale_rough),
                    init.gridalize(2, scale_rough), 'obst_UGV')
    obst_person1 = Obstacle(init.gridalize((3, 2.5, 0), scale_rough), init.gridalize(1, scale_rough),
                    mapBound_grid_rough[2], 'obst_person') # mapHeight_grid is already gridalized

    obst_UAV1.setRange(mapBound_grid_rough[2])
    obst_UAV2.setRange(mapBound_grid_rough[2])
    obst_UAV3.setRange(mapBound_grid_rough[2])
    obst_UGV1.setRange(mapBound_grid_rough[2])
    obst_UGV2.setRange(mapBound_grid_rough[2])
    obst_person1.setRange(mapBound_grid_rough[2])

    obstArray = [obst_UAV1, obst_UAV2, obst_UAV3, obst_UGV1, obst_UGV2, obst_person1]
    '''

    heap_percolation = 0

    print '====================================='
    print 'Generating the rough path...'
    print 'startPoint: ', startPoint
    print 'endPoint: ', endPoint

    for item in obstArray_rough:
        if item.conflict(endPoint):
            print
            rospy.logwarn('Destination conflicts with obstacle!')
            rospy.signal_shutdown()
    else:
        # Plan the path
        start_time = timeit.default_timer()
        came_from, cost_so_far = a_star_search(diagram_rough, obstArray_rough, startPoint, endPoint)
        roughTrajectory = reconstruct_path(came_from, start=startPoint, goal=endPoint)
        execution_time = timeit.default_timer() - start_time

        searchedPoints = []
        for key in came_from:
            searchedPoints.append(came_from[key])
        searchedPoints.remove(None)
        # Performance measurement
        len_of_path = cost_so_far[endPoint]
        vertex_expension = len(searchedPoints)

        print
        print 'Length of path: ', float(len_of_path / scale_rough)
        print
        print 'Path planning execution time: ', execution_time
        print
        print 'Vetices expanded: ', vertex_expension
        print
        print 'Heap percolated: ', heap_percolation

    return roughTrajectory


def generateRefinedPath():
    global mapBound_grid_fine
    global diagram_fine
    global current_position
    global scale_rough
    global scale_fine
    global obstArray_fine
    global heap_percolation
    global i
    global scaleRatio
    global roughTrajectory

    # initialize environment
    startPoint = tuple(init.gridalize(current_position, scale_fine))
    # (roughTrajectory[i-2][0]*scaleRatio, roughTrajectory[i-2][1]*scaleRatio, roughTrajectory[i-2][2]*scaleRatio)
    '''furtherst point in range in roughTrajectory'''
    distsq_min = 1000
    closestPoint_id = 0
    for i in range(len(roughTrajectory)):
        distsq = (roughTrajectory[i][0]*scaleRatio -startPoint[0])**2 + (roughTrajectory[i][1]*scaleRatio -startPoint[1])**2 + (roughTrajectory[i][2]*scaleRatio -startPoint[2])**2
        if distsq < distsq_min:
            distsq_min = distsq
            closestPoint_id = i

    endPoint = (roughTrajectory[closestPoint_id+2][0] *scaleRatio, roughTrajectory[closestPoint_id+2][1] *scaleRatio,
                roughTrajectory[closestPoint_id+2][2] *scaleRatio)
    # (roughTrajectory[i][0]*scaleRatio, roughTrajectory[i][1]*scaleRatio, roughTrajectory[i][2]*scaleRatio)

    '''
    # initialize obstacles with position of their centre point
    obst_UAV1 = Obstacle(init.gridalize((0.5, 2.5, 1), scale_fine), init.gridalize(0.25, scale_fine),
                    init.gridalize(2, scale_fine), 'obst_UAV')
    obst_UAV2 = Obstacle(init.gridalize((1.5, 0.5, 1.5), scale_fine), init.gridalize(0.25, scale_fine),
                    init.gridalize(2, scale_fine), 'obst_UAV')
    obst_UAV3 = Obstacle(init.gridalize((3.5, 4.5, 2), scale_fine), init.gridalize(0.25, scale_fine),
                    init.gridalize(2, scale_fine), 'obst_UAV')
    obst_UGV1 = Obstacle(init.gridalize((1.5, 4.5, 2), scale_fine), init.gridalize(0.5, scale_fine),
                    init.gridalize(2, scale_fine), 'obst_UGV')
    obst_UGV2 = Obstacle(init.gridalize((4.5, 2.5, 2), scale_fine), init.gridalize(0.5, scale_fine),
                    init.gridalize(2, scale_fine), 'obst_UGV')
    obst_person1 = Obstacle(init.gridalize((3, 2.5, 0), scale_fine), init.gridalize(1, scale_fine),
                    mapBound_grid_fine[2], 'obst_person') # mapHeight_grid is already gridalized
    # obst_UAV1 = Obstacle(init.gridalize((random.uniform(0.25,1.75), random.uniform(2.25,3.75), random.uniform(1,3)), scale_fine), 'obst_UAV')
    # obst_UAV2 = Obstacle(init.gridalize((random.uniform(0.25,1.75), random.uniform(0.25,1.75), random.uniform(1,3)), scale_fine), 'obst_UAV')
    # obst_UAV3 = Obstacle(init.gridalize((random.uniform(3.25,4.75), random.uniform(3.25,4.75), random.uniform(1,3)), scale_fine), 'obst_UAV')
    # obst_UGV1 = Obstacle(init.gridalize((random.uniform(0.5,2.5), 4.5, 2), scale_fine), 'obst_UGV')
    # obst_UGV2 = Obstacle(init.gridalize((4.5, random.uniform(0.5,2.5), 2), scale_fine), 'obst_UGV')
    # obst_person1 = Obstacle(init.gridalize((3, random.uniform(1,3), 0), scale_fine), 'obst_person')
    # set the size of obstacles
    obst_UAV1.setRange(mapBound_grid_fine[2])
    obst_UAV2.setRange(mapBound_grid_fine[2])
    obst_UAV3.setRange(mapBound_grid_fine[2])
    obst_UGV1.setRange(mapBound_grid_fine[2])
    obst_UGV2.setRange(mapBound_grid_fine[2])
    obst_person1.setRange(mapBound_grid_fine[2])

    obstArray = [obst_UAV1, obst_UAV2, obst_UAV3, obst_UGV1, obst_UGV2, obst_person1]
    '''

    # diagram.walls = list(set(diagram.walls + obst_UAV1.points + obst_UAV2.points +
    #                     obst_UAV3.points + obst_UGV1.points + obst_UGV2.points +
    #                     obst_person1.points))


    heap_percolation = 0
    print '-------------------------------------'
    print 'Generating the refined path...'
    print 'startPoint: ', startPoint
    print 'endPoint: ', endPoint

    boundMarker     = visualization.setBoundary(mapBound_grid_fine)
    obstMarkerArray = visualization.setObstacle(obstArray_fine)

    # Publish the boundary and obstacle
    boundPub.publish(boundMarker)
    obstPub.publish(obstMarkerArray)

    for item in obstArray_fine:
        if item.conflict(endPoint):
        # if item.conflict(endPoint) or item.conflict(startPoint):
            print
            print 'Destination conflicts with obstacle!'
            # print 'Starting point/ destination conflicts with obstacle!'
            ''' need to be modified '''
            target_point = (int(random.uniform(2,3)*scale_fine), int(random.uniform(2,3)*scale_fine),
                                int(random.uniform(1,2)*scale_fine))
            break
    else:
        # Plan the path
        print
        print 'Planning path...'
        start_time = timeit.default_timer()
        came_from, cost_so_far = a_star_search(diagram_fine, obstArray_fine, startPoint, endPoint)
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
        print
        print 'Path planning execution time: ', execution_time
        print
        print 'Vetices expanded: ', vertex_expension
        print
        print 'Heap percolated: ', heap_percolation

    return finalTrajectory


###############################################################################
###############################################################################
''' Set original environment '''
mapBound_metre = (4, 4, 2.5)      # 3D boundary of the operating environment
scale_rough = 4
scale_fine = 100
scaleRatio = float(scale_fine/scale_rough)
refineRatio = int((scale_fine/scale_rough)**(1.0/3))

startPoint  = (0, 0, 0)
endPoint    = (0, 0, 0)
current_position  = (0, 0, 0)
target_position   = (4, 4, 2.5)

''' Initialize ROS node and publishers '''
rospy.init_node('astar_node', anonymous=True) # rosnode name
(pathPub, pointsPub, boundPub, obstPub) = init.initPublishers() # initialize publishers
rospy.sleep(0.3) # it takes time to initialize publishers
rate = rospy.Rate(1) # loop runs at x Hertz

''' Set original value of flags '''
gotPath = False
pathBlocked = True
callback_obst_flg = True
callback_current_flg = True
callback_target_flg = True

i = 2

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
obstArray_rough = []
obstArray_fine  = []

''' Loop for path planning '''
while not rospy.is_shutdown():

    # receive obstacle postion
    while callback_obst_flg:
        obstSub = rospy.Subscriber('/UAV_2/pose', PoseStamped, callback_obst)
        callback_obst_flg = False
    # receive current position of UAV
    while callback_current_flg:
        ppSub   = rospy.Subscriber('/UAV_1/pose', PoseStamped, callback_current)
        callback_current_flg = False
    # receive requested destination
    while callback_target_flg:
        ppSub   = rospy.Subscriber('target_request', PoseStamped, callback_target)
        callback_target_flg = False

    startPoint  = init.gridalize(current_position, scale_rough)
    endPoint    = init.gridalize(target_position, scale_rough)

    if (not gotPath) or pathBlocked:
        roughTrajectory = generateRoughPath() # plan a new rough path
        gotPath = True
        pathBlocked = False

    '''Start to refine the path...'''
    finalTrajectory = generateRefinedPath()

    i += 2
    rospy.sleep(1.)
        # rate.sleep()
        # rospy.signal_shutdown()
