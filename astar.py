import rospy
import collections
import heapq
import random
import initialization
import visualization
import math
from geometry_msgs.msg import PointStamped, Point, PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]


class SquareGrid(object):
    def __init__(self, length, width, height):
        self.length = length
        self.width  = width
        self.height = height
        self.walls = []

    def in_bounds(self, id):
        (x, y, z) = id
        return 0 <= x < self.length and 0 <= y < self.width and 0<= z<= self.height

    def passable(self, id):
        return id not in self.walls

    def neighbors(self, id):
        (x, y, z) = id
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


def generateObstacle(centre_point):
    length  = random.uniform(0.5, 1)  # length of obstacle 0.5 ~ 1.0 metres
    width   = random.uniform(0.2, 0.3) # width of obstacle 0.2 ~ 0.3 metres
    height  = random.uniform(0.2, 0.3) # height of obstacle 0.2 ~ 0.3 metres
    if random.uniform(0, 1) > 0.5:
        temp = length
        length = width
        width = temp
    if random.uniform(0, 1) > 0.5:
        temp = length
        length = height
        height = temp
    length_grid = int(length*scale)
    width_grid  = int(width*scale)
    height_grid  = int(height*scale)
    centre_grid = []
    centre_grid.append(int(centre_point[0]*scale))
    centre_grid.append(int(centre_point[1]*scale))
    centre_grid.append(int(centre_point[2]*scale))
    obstacle = []
    for i in range(max(0, centre_grid[0] - length_grid/2),
            min(length_of_map, centre_grid[0] + length_grid/2)):
        for j in range(max(0, centre_grid[1] - width_grid/2),
                min(width_of_map, centre_grid[1] + width_grid/2)):
            for k in range(max(0, centre_grid[2] - height_grid),
                    min(height_of_map, centre_grid[2])):
                obstacle.append((i, j, k))
    return obstacle


def reconstruct_path(came_from, start, goal):
    current = goal
    path = [current]
    while current != start:
        if current in came_from:
            current = came_from[current]
            path.append(current)
        else:
            rospy.logfatal('The destination has been surrounded by obstacles! No available path!')
            print 'goalpoint: ', goal
            print
            print 'currentpoint: ', start
            print
            print 'walls: \n', diagram.walls
            print
            rospy.signal_shutdown()
    # path.append(start) # optional
    path.reverse() # optional
    return path


def heuristic(a, b):
    (x1, y1, z1) = a
    (x2, y2, z2) = b
    return math.sqrt((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)


def a_star_search(graph, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0

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
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current

    return came_from, cost_so_far


def callback_obst(centre_point):
    # rospy.logwarn(len(diagram.walls))
    # rospy.logwarn((centre_point.pose.position.x, centre_point.pose.position.y,
    #                 centre_point.pose.position.z))
    diagram.walls = list(set(diagram.walls +
                        generateObstacle((centre_point.pose.position.x,
                        centre_point.pose.position.y, centre_point.pose.position.z))))
    callback_obst_flg = True


def callback_start(data):  # data contains the current point
    # rospy.loginfo((data.pose.position.x, data.pose.position.y, data.pose.position.z))
    global current_point
    current_point = (int(data.pose.position.x * scale), int(data.pose.position.y * scale),
                        int(data.pose.position.z* scale))
    callback_start_flg = True


def callback_end(data):  # data contains the target point
    # rospy.loginfo((data.points[1].x, data.points[1].y, data.points[1].z))
    global target_point
    target_point = (int(data.pose.position.x * scale), int(data.pose.position.y * scale),
                        int(data.pose.position.z* scale))
    callback_end_flg = True


###############################################################################
# try:

# Initialization
scale = 10

length_of_map = int(5*scale)
width_of_map = int(5*scale)
height_of_map = int(5*scale)
current_point = (int(0*scale), int(0*scale), int(0*scale))
target_point = (int(random.uniform(2,3)*scale), int(random.uniform(2,3)*scale), int(random.uniform(1,2)*scale))

diagram = GridWithWeights(length_of_map, width_of_map, height_of_map)
diagram.walls = []
# diagram.walls = generateObstacle((1.5, 1.5, 1.5))

callback_obst_flg = True
callback_start_flg = True
callback_end_flg = True

# Loop for path planning
while not rospy.is_shutdown():
    rospy.sleep(1.)
    start_point = current_point
    end_point   = target_point
    print
    print 'start_point: ', start_point
    print
    print 'end_point: ', end_point
    rospy.init_node('astar_node', anonymous=True) # rosnode name
    # rate = rospy.Rate(1)

    # receive obstacle postion
    while callback_obst_flg:
        obstSub = rospy.Subscriber('obst_request', PoseStamped, callback_obst)
        callback_obst_flg = False
    # receive current position of UAV
    # while callback_start_flg:
    #     ppSub   = rospy.Subscriber('/UAV_1/pose', PoseStamped, callback_start)
    #     callback_start_flg = False
    # receive requested destination
    while callback_end_flg:
        ppSub   = rospy.Subscriber('end_request', PoseStamped, callback_end)
        callback_end_flg = False

    boundary = visualization.setBoundary(length_of_map, width_of_map, height_of_map)
    obstacle = visualization.setObstacle(diagram.walls)

    (pathPub, obstPub, pointsPub) = initialization.initPublishers()

    # for point in diagram.walls:
    if end_point in diagram.walls:
    # if start_point in diagram.walls or end_point in diagram.walls:
        print
        print 'Destination conflicts with obstacle!'
        # print 'Starting point/ destination conflicts with obstacle!'
        target_point = (int(random.uniform(2,3)*scale), int(random.uniform(2,3)*scale),
                            int(random.uniform(1,2)*scale))
        break

    else:
        # Plan the path
        print
        print 'Planning path...'
        came_from, cost_so_far = a_star_search(diagram, start_point, end_point)
        finalTrajectory = reconstruct_path(came_from, start=start_point, goal=end_point)

        # These four values are all visualization markers!
        (sourcePoint, goalPoint, neighbourPoint,
            finalPath) = visualization.setPathMarkers(finalTrajectory, came_from)
        finalPath.text = '0'

        obstPub.publish(boundary)
        obstPub.publish(obstacle)
        pointsPub.publish(sourcePoint)
        pointsPub.publish(goalPoint)
        pointsPub.publish(neighbourPoint)
        pathPub.publish(finalPath)

        print
        print 'Path sent.'

        #rospy.sleep(10.)
        # rate.sleep()

# except KeyboardInterrupt:
#     rospy.logfatal('ahahah')
#     print 'goalpoint: ', goal
#     print
#     print 'walls: \n', diagram.walls
#     print
#     sys.exit()
