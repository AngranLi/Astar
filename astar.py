import rospy
import collections
import heapq
import random
import initialization
import visualization
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class Queue:
    def __init__(self):
        self.elements = collections.deque()

    def empty(self):
        return len(self.elements) == 0

    def put(self, x):
        self.elements.append(x)

    def get(self):
        return self.elements.popleft()

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
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.walls = []

    def in_bounds(self, id):
        (x, y) = id
        return 0 <= x < self.width and 0 <= y < self.height

    def passable(self, id):
        return id not in self.walls

    def neighbors(self, id):
        # rospy.logfatal(id)
        (x, y) = id
        # results = [(x+1, y), (x, y-1), (x-1, y), (x, y+1)]
        results = [(x+1, y), (x+1, y-1), (x, y-1), (x-1, y-1), (x-1, y), (x-1, y+1), (x, y+1), (x+1, y+1)]
        if (x + y) % 2 == 0: results.reverse() # aesthetics [Attention!]
        results = filter(self.in_bounds, results)
        results = filter(self.passable, results)
        return results

class GridWithWeights(SquareGrid):
    def __init__(self, width, height):
        super(GridWithWeights, self).__init__(width, height)
        self.weights = {}

    def cost(self, from_node, to_node):
        return self.weights.get(to_node, 1) # ???

def generateObstacle(centre_point):
    height = random.uniform(0.5, 1) # height of obstacle 1 ~ 2 metres
    width = random.uniform(1, 2)    # width of obstacle 1 ~ 2.5 metres
    height_grid = int(height*scale)
    width_grid = int(width*scale)
    centre_grid = []
    centre_grid.append(int(centre_point[0]))
    centre_grid.append(int(centre_point[1]))
    obstacle = []
    for i in range(max(0, centre_grid[0] - height_grid/2), min(height_of_map, centre_grid[0] + height_grid/2)):
        for j in range(max(0, centre_grid[1] - width_grid/2), min(width_of_map, centre_grid[1] + width_grid/2)):
            obstacle.append((j,i))
    return obstacle

# def from_id_width(id, width):
#     return (id % width, id // width)
#
# DIAGRAM1_WALLS = [from_id_width(id, width=30) for id in [21,22,51,52,81,82,93,94,
# 111,112,123,124,133,134,141,142,153,154,163,164,171,172,173,174,175,183,184,193,194,
# 201,202,203,204,205,213,214,223,224,243,244,253,254,273,274,283,284,303,304,313,314,
# 333,334,343,344,373,374,403,404,433,434]]

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
    for y in range(graph.height):
        for x in range(graph.width):
            print("%%-%ds" % width % draw_tile(graph, (x, y), style, width)), #, end="")
        print()

def reconstruct_path(came_from, start, goal):
    current = goal
    path = [current]
    while current != start:
        current = came_from[current]
        path.append(current)
    # path.append(start) # optional
    path.reverse() # optional
    return path

def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)

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
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current

    return came_from, cost_so_far

##########################################################
# Initialization
scale = 10

height_of_map = int(3*scale)
width_of_map = int(6*scale)
start_point = (int(0*scale), int(0*scale))
# end_point = (int(0*scale), int(0*scale))
end_point = (int(random.uniform(4,6)*scale), int(random.uniform(2,3)*scale))
# centre_of_rectangle_obstacle = (int(random.uniform(0,3)*scale), int(random.uniform(0,6)*scale))

diagram = GridWithWeights(width_of_map, height_of_map)
diagram.walls = []
# diagram.walls = generateObstacle(centre_of_rectangle_obstacle)


print 'end_point: ', end_point
print

rospy.init_node('markers_pub', anonymous=True) # rosnode name
obstPub = rospy.Publisher('obst_markers', Marker, queue_size=10)

boundary = visualization.setBoundary(height_of_map, width_of_map)
obstacle = visualization.setObstacle(diagram.walls)


for point in diagram.walls:
    if point == start_point or point == end_point:
        print 'Starting point / destination conflicts with obstacle!'
        break
else:
    # Plan the path
    came_from, cost_so_far = a_star_search(diagram, start_point, end_point)
    finalTrajectory = reconstruct_path(came_from, start=start_point, goal=end_point)

    # These four values are all visualization markers!
    (sourcePoint, goalPoint, neighbourPoint,
        finalPath) = visualization.setPathMarkers(finalTrajectory, came_from)

    rate = rospy.Rate(100)
    pathPub = initialization.initPublishers()
    while not rospy.is_shutdown():
        pathPub.publish(boundary)
        pathPub.publish(obstacle)
        pathPub.publish(sourcePoint)
        pathPub.publish(goalPoint)
        pathPub.publish(neighbourPoint)
        pathPub.publish(finalPath)
        rate.sleep()

    # rospy.spin()

    print
    draw_grid(diagram, width=1, point_to=came_from, start=start_point, goal=end_point)
    # print()
    # draw_grid(diagram, width=1, number=cost_so_far, start=start_point, goal=end_point)
    # print()
    # draw_grid(diagram, width=1, path=finalTrajectory)
