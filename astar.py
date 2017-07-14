import rospy
import collections
import heapq
import random
import initialization
import visualization
import math
from geometry_msgs.msg import PointStamped, Point
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
    def __init__(self, length, width):
        self.length = length
        self.width = width
        self.walls = []

    def in_bounds(self, id):
        (x, y) = id
        return 0 <= x < self.length and 0 <= y < self.width

    def passable(self, id):
        return id not in self.walls

    def neighbors(self, id):
        (x, y) = id
        # results = [(x+1, y), (x, y-1), (x-1, y), (x, y+1)]
        results = [(x+1, y), (x+1, y-1), (x, y-1), (x-1, y-1), (x-1, y),
                    (x-1, y+1), (x, y+1), (x+1, y+1)]
        if (x + y) % 2 == 0: results.reverse() # aesthetics [Attention!]
        results = filter(self.in_bounds, results)
        results = filter(self.passable, results)
        return results


class GridWithWeights(SquareGrid):
    def __init__(self, length, width):
        super(GridWithWeights, self).__init__(length, width)
        self.weights = {}

    def cost(self, from_node, to_node):
        return self.weights.get(to_node, 1) # default is 1


def generateObstacle(centre_point):
    length  = random.uniform(0.5, 1)  # length of obstacle 1 ~ 2.5 metres
    width   = random.uniform(0.2, 0.3) # width of obstacle 1 ~ 2 metres
    if random.uniform(0, 1) > 0.5:
        temp = length
        length = width
        width = temp
    length_grid = int(length*scale)
    width_grid  = int(width*scale)
    centre_grid = []
    centre_grid.append(int(centre_point[0]*scale))
    centre_grid.append(int(centre_point[1]*scale))
    obstacle = []
    for i in range(max(0, centre_grid[0] - length_grid/2), min(length_of_map, centre_grid[0] + length_grid/2)):
        for j in range(max(0, centre_grid[1] - width_grid/2), min(width_of_map, centre_grid[1] + width_grid/2)):
            obstacle.append((i,j))
    return obstacle


def draw_tile(graph, id, style, width):
    # rospy.logfatal(id)
    r = "."
    if 'number' in style and id in style['number']: r = "%d" % style['number'][id]
    if 'point_to' in style and style['point_to'].get(id, None) is not None:
        (x1, y1) = id
        (x2, y2) = style['point_to'][id]
        if x2 == x1 + 1:
            if y2 == y1 + 1:
                r = u'\u2198'
            elif y2 == y1 - 1:
                r = u'\u2197'
            else:
                r = u"\u2192"
        if x2 == x1 - 1:
            if y2 == y1 + 1:
                r = u'\u2199'
            elif y2 == y1 - 1:
                r = u'\u2196'
            else:
                r = u"\u2190"
        if x2 == x1:
            if y2 == y1 + 1:
                r = u"\u2193"
            else :
                r = u"\u2191" # y2 == y1 - 1
    if 'start' in style and id == style['start']: r = "A"
    if 'goal' in style and id == style['goal']: r = "Z"
    if 'path' in style and id in style['path']: r = "@"
    if id in graph.walls: r = "#" * width
    return r


def draw_grid(graph, width=2, **style):
    # print style
    for y in range(graph.width):
        for x in range(graph.length):
            print("%%-%ds" % width % draw_tile(graph, (x, y), style, width)),
        print()


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
            print 'walls: \n', diagram.walls
            print
            rospy.signal_shutdown()
    # path.append(start) # optional
    path.reverse() # optional
    return path


def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)


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
            # if the neighbourPoint and the current are on a diagonal
            if abs(next[0]-current[0]) + abs(next[1]-current[1]) == 2:
                new_cost = cost_so_far[current] + math.sqrt(2)*graph.cost(current, next)
            else:
                new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current

    return came_from, cost_so_far


# [Problem!! multiple call!!]
def callback_obst(centre_point):
    # rospy.logwarn(len(diagram.walls))
    rospy.loginfo((centre_point.points[0].x, centre_point.points[0].y))
    diagram.walls = list(set(diagram.walls +
        generateObstacle((centre_point.points[0].x, centre_point.points[0].y))))
    callback_obst_flg = True


def callback_pp(data):  # data contains the current and target points
    rospy.logwarn((data.points[1].x, data.points[1].y))
    global current_point
    global target_point
    current_point = (int(data.points[0].x * scale), int(data.points[0].y * scale))
    target_point   = (int(data.points[1].x * scale), int(data.points[1].y * scale))
    callback_pp_flg = True


##########################################################
# try:

# Initialization
scale = 10

length_of_map = int(6*scale)
width_of_map = int(3*scale)
current_point = (int(0*scale), int(0*scale))
# target_point = (29, 19)
target_point = (int(random.uniform(4,6)*scale), int(random.uniform(2,3)*scale))

diagram = GridWithWeights(length_of_map, width_of_map)
diagram.walls = []
# diagram.walls = [(27,18), (27,19), (27,20), (27,21), (27,22), (27,23),
#                     (28,18), (28,19), (28,20), (28,21), (28,22), (28,23),
#                     (29,15), (30,15), (31,15), (32,15), (33,15), (34,15),
#                     (29,16), (30,16), (31,16), (32,16), (33,16), (34,16)]

callback_obst_flg = True
callback_pp_flg = True

# Loop for path planning
while not rospy.is_shutdown():
    start_point = current_point
    end_point   = target_point
    print
    print 'start_point: ', start_point
    print
    print 'end_point: ', end_point
    rospy.init_node('astar_node', anonymous=True) # rosnode name
    rate = rospy.Rate(1)

    while callback_obst_flg:
        obstSub = rospy.Subscriber('obst_request', Marker, callback_obst)
        callback_obst_flg = False
    while callback_pp_flg:
        ppSub   = rospy.Subscriber('pp_request', Marker, callback_pp)
        callback_pp_flg = False

    obstPub = rospy.Publisher('obst_markers', Marker, queue_size=10)

    boundary = visualization.setBoundary(length_of_map, width_of_map)
    obstacle = visualization.setObstacle(diagram.walls)
    # print 'diagram.walls: \n', diagram.walls

    for point in diagram.walls:
        if point == start_point or point == end_point:
            print
            print 'Starting point / destination conflicts with obstacle!'
            target_point = (int(random.uniform(4,6)*scale), int(random.uniform(2,3)*scale))
            break

    else:
        # Plan the path
        came_from, cost_so_far = a_star_search(diagram, start_point, end_point)
        finalTrajectory = reconstruct_path(came_from, start=start_point, goal=end_point)


        # These four values are all visualization markers!
        (sourcePoint, goalPoint, neighbourPoint,
            finalPath) = visualization.setPathMarkers(finalTrajectory, came_from)

        pathPub = initialization.initPublishers()

        pathPub.publish(boundary)
        pathPub.publish(obstacle)
        pathPub.publish(sourcePoint)
        pathPub.publish(goalPoint)
        pathPub.publish(neighbourPoint)
        pathPub.publish(finalPath)

        draw_grid(diagram, width=1, point_to=came_from, start=start_point, goal=end_point)
        # print()
        # draw_grid(diagram, width=1, number=cost_so_far, start=start_point, goal=end_point)
        # print()
        # draw_grid(diagram, width=1, path=finalTrajectory)

    rate.sleep()

# except KeyboardInterrupt:
#     rospy.logfatal('ahahah')
#     print 'goalpoint: ', goal
#     print
#     print 'walls: \n', diagram.walls
#     print
#     sys.exit()
