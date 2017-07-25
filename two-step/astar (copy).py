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
    def __init__(self, length, width, height):
        self.length = length
        self.width  = width
        self.height = height
        # self.walls = [] # !!! Stop using diagram.walls to represent obstacles

    def in_bounds(self, point):
        (x, y, z) = point
        return 0 <= x <= self.length and 0 <= y <= self.width and 0<= z <= self.height

    def passable(self, point):
        global obstArray
        for i in range(len(obstArray)):
            if obstArray[i].conflict(point):
                return False
        else:
            return True
        # return point not in self.walls

    def neighbors(self, point):
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
        results = filter(self.passable, results)
        return results


class GridWithWeights(SquareGrid):
    def __init__(self, length, width, height):
        super(GridWithWeights, self).__init__(length, width, height)
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

    def setRange(self):
        if self.text == 'obst_UAV':
            self.top    = min(height_of_map, self.centre_point[2] + self.height/2)
            self.bottom = max(0, self.centre_point[2] - self.height/2)
        elif self.text == 'obst_UGV':
            self.top    = min(height_of_map, self.centre_point[2])
            self.bottom = max(0, self.centre_point[2] - self.height)
        elif self.text == 'obst_person':
            self.top    = height_of_map
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


def pathBlocked(obstArray, finalTrajectory):
    for item in obstArray:
        for point in finalTrajectory:
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


def a_star_search(graph, start, goal):
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

        for next in graph.neighbors(current):
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


# def callback_obst(centre_point):
#     # rospy.logwarn(len(diagram.walls))
#     # rospy.logwarn((centre_point.pose.position.x, centre_point.pose.position.y,
#     #                 centre_point.pose.position.z))
#     diagram.walls = list(set(diagram.walls +
#                         makeObstacle((centre_point.pose.position.x,
#                         centre_point.pose.position.y, centre_point.pose.position.z))))
#     callback_obst_flg = True


# def callback_start(data):  # data contains the current point
#     # rospy.loginfo((data.pose.position.x, data.pose.position.y, data.pose.position.z))
#     global current_point
#     current_point = (int(data.pose.position.x * scale), int(data.pose.position.y * scale),
#                         int(data.pose.position.z* scale))
#     callback_start_flg = True
#
#
# def callback_end(data):  # data contains the target point
#     # rospy.loginfo((data.points[1].x, data.points[1].y, data.points[1].z))
#     global target_point
#     target_point = (int(data.pose.position.x * scale), int(data.pose.position.y * scale),
#                         int(data.pose.position.z* scale))
#     callback_end_flg = True

def roughPath():
    global length_of_map
    global width_of_map
    global height_of_map
    global current_point
    global target_point
    global scale_rough
    global obstArray
    global heap_percolation
    global roughTrajectory

    # initialize environment
    (length_of_map, width_of_map, height_of_map) = init.gridalize((5, 5, 3), scale_rough)
    current_point   = tuple(init.gridalize((0, 0, 0), scale_rough))
    target_point    = tuple(init.gridalize((5, 5, 3), scale_rough))

    diagram = GridWithWeights(length_of_map, width_of_map, height_of_map)

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
                    height_of_map, 'obst_person') # height_of_map is already gridalized

    obst_UAV1.setRange()
    obst_UAV2.setRange()
    obst_UAV3.setRange()
    obst_UGV1.setRange()
    obst_UGV2.setRange()
    obst_person1.setRange()

    obstArray = [obst_UAV1, obst_UAV2, obst_UAV3, obst_UGV1, obst_UGV2, obst_person1]

    heap_percolation = 0
    start_point = current_point
    end_point   = target_point
    print '====================================='
    print 'Generating the first rough path'
    print
    print 'start_point: ', start_point
    print
    print 'end_point: ', end_point

    for item in obstArray:
        if item.conflict(end_point):
            print
            rospy.logwarn('Destination conflicts with obstacle!')
            # target_point = (int(random.uniform(2,3)*scale_rough), int(random.uniform(2,3)*scale_rough),
            #                     int(random.uniform(1,2)*scale_rough))
            rospy.signal_shutdown()
    else:
        # Plan the path
        start_time = timeit.default_timer()
        came_from, cost_so_far = a_star_search(diagram, start_point, end_point)
        roughTrajectory = reconstruct_path(came_from, start=start_point, goal=end_point)
        execution_time = timeit.default_timer() - start_time

        searchedPoints = []
        for key in came_from:
            searchedPoints.append(came_from[key])
        searchedPoints.remove(None)
        # Performance measurement
        len_of_path = cost_so_far[end_point]
        vertex_expension = len(searchedPoints)

        print
        print 'Length of path: ', len_of_path
        print
        print 'Path planning execution time: ', execution_time
        print
        print 'Vetices expanded: ', vertex_expension
        print
        print 'Heap percolated: ', heap_percolation



###############################################################################
###############################################################################
gotPath = False
scale_rough = 4
scale_fine = 100
scaleRatio = int(scale_fine/scale_rough)
refineRatio = int((scale_fine/scale_rough)**(1.0/3))

current_point   = tuple(init.gridalize((0, 0, 0), scale_fine))
target_point    = tuple(init.gridalize((0, 0, 0), scale_fine))
i = 2

# Performance measurement
len_of_path = 0
execution_time = 0
vertex_expension = 0

# Initialization
rospy.init_node('astar_node', anonymous=True) # rosnode name
(pathPub, pointsPub, boundPub, obstPub) = init.initPublishers() # initialize publishers
rospy.sleep(0.3) # it takes time to initialize publishers
rate = rospy.Rate(1) # loop runs at x Hertz

while not rospy.is_shutdown():
    if gotPath:
        if pathBlocked(obstArray, finalTrajectory):
            roughPath() # plan a new rough path
        else:
            pass
    else:
            roughPath()
            gotPath = True

###########################################
# Generate the refined path

    print
    print 'Start to refine the path...'

    # initialize environment
    (length_of_map, width_of_map, height_of_map) = init.gridalize((5, 5, 3), scale_fine)
    current_point   = target_point
    target_point    = (roughTrajectory[i][0]*scaleRatio, roughTrajectory[i][1]*scaleRatio,
                        roughTrajectory[i][2]*scaleRatio)
    # target_point    = (roughTrajectory[refineRatio][0]*scaleRatio, roughTrajectory[refineRatio][1]*scaleRatio,
    #                     roughTrajectory[refineRatio][2]*scaleRatio)

    diagram = GridWithWeights(length_of_map, width_of_map, height_of_map)

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
                    height_of_map, 'obst_person') # height_of_map is already gridalized
    # obst_UAV1 = Obstacle(init.gridalize((random.uniform(0.25,1.75), random.uniform(2.25,3.75), random.uniform(1,3)), scale_fine), 'obst_UAV')
    # obst_UAV2 = Obstacle(init.gridalize((random.uniform(0.25,1.75), random.uniform(0.25,1.75), random.uniform(1,3)), scale_fine), 'obst_UAV')
    # obst_UAV3 = Obstacle(init.gridalize((random.uniform(3.25,4.75), random.uniform(3.25,4.75), random.uniform(1,3)), scale_fine), 'obst_UAV')
    # obst_UGV1 = Obstacle(init.gridalize((random.uniform(0.5,2.5), 4.5, 2), scale_fine), 'obst_UGV')
    # obst_UGV2 = Obstacle(init.gridalize((4.5, random.uniform(0.5,2.5), 2), scale_fine), 'obst_UGV')
    # obst_person1 = Obstacle(init.gridalize((3, random.uniform(1,3), 0), scale_fine), 'obst_person')
    # set the size of obstacles
    obst_UAV1.setRange()
    obst_UAV2.setRange()
    obst_UAV3.setRange()
    obst_UGV1.setRange()
    obst_UGV2.setRange()
    obst_person1.setRange()

    obstArray = [obst_UAV1, obst_UAV2, obst_UAV3, obst_UGV1, obst_UGV2, obst_person1]

    # diagram.walls = list(set(diagram.walls + obst_UAV1.points + obst_UAV2.points +
    #                     obst_UAV3.points + obst_UGV1.points + obst_UGV2.points +
    #                     obst_person1.points))

    # callback_obst_flg = True
    # callback_start_flg = True
    # callback_end_flg = True

    heap_percolation = 0
    start_point = current_point
    end_point   = target_point
    print
    print 'refineRatio: ', refineRatio
    print
    print 'start_point: ', start_point
    print
    print 'end_point: ', end_point

    # # receive obstacle postion
    # while callback_obst_flg:
    #     obstSub = rospy.Subscriber('obst_request', PoseStamped, callback_obst)
    #     callback_obst_flg = False
    # # receive current position of UAV
    # while callback_start_flg:
    #     ppSub   = rospy.Subscriber('/UAV_1/pose', PoseStamped, callback_start)
    #     callback_start_flg = False
    # # receive requested destination
    # while callback_end_flg:
    #     ppSub   = rospy.Subscriber('end_request', PoseStamped, callback_end)
    #     callback_end_flg = False

    boundMarker     = visualization.setBoundary(length_of_map, width_of_map, height_of_map)
    obstMarkerArray = visualization.setObstacle(obstArray)
    # rospy.logfatal('1st Obtacle')
    # print obstMarkerArray.markers[0]
    # rospy.logfatal('2nd Obtacle')
    # print obstMarkerArray.markers[1]

    # Publish the boundary and obstacle
    boundPub.publish(boundMarker)
    obstPub.publish(obstMarkerArray)

    for item in obstArray:
        if item.conflict(end_point):
        # if item.conflict(end_point) or item.conflict(start_point):
            print
            print 'Destination conflicts with obstacle!'
            # print 'Starting point/ destination conflicts with obstacle!'
            target_point = (int(random.uniform(2,3)*scale_fine), int(random.uniform(2,3)*scale_fine),
                                int(random.uniform(1,2)*scale_fine))
            break
    else:
        # Plan the path
        print
        print 'Planning path...'
        start_time = timeit.default_timer()
        came_from, cost_so_far = a_star_search(diagram, start_point, end_point)
        finalTrajectory = reconstruct_path(came_from, start=start_point, goal=end_point)
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
        len_of_path = cost_so_far[end_point]
        vertex_expension = len(neighbourPoint.points)
        # f = open('cost_so_far', 'w')
        # f.write(str(cost_so_far))
        # f.close()
        print
        print 'Length of path: ', len_of_path
        print
        print 'Path planning execution time: ', execution_time
        print
        print 'Vetices expanded: ', vertex_expension
        print
        print 'Heap percolated: ', heap_percolation

        i += 2
        rospy.sleep(5.)
        # rate.sleep()
        # rospy.signal_shutdown()
