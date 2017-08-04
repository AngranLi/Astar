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
        return self.weights.get(to_node, 1) # ???


def draw_tile(graph, id, style, width):
    # rospy.logfatal(id)
    r = "."
    if 'number' in style and id in style['number']: r = "%d" % style['number'][id]
    if 'start' in style and id == style['start']: r = "A"
    if 'goal' in style and id == style['goal']: r = "Z"
    if 'path' in style and id in style['path']: r = "@"
    if id in graph.walls: r = "#" * width
    return r


def draw_grid(graph, width=2, **style):
    # print style
    for y in range(30):
        for x in range(60):
            print("%%-%ds" % width % draw_tile(graph, (x, y), style, width)),
        print()

#################################################################
diagram = GridWithWeights(60, 30)
diagram.walls = [(20, 25), (21, 28), (17, 20), (56, 25), (46, 12), (18, 19), (42, 20), (23, 26), (45, 9), (43, 25), (58, 19), (35, 15), (59, 26), (35, 21), (12, 17), (51, 22), (39, 11), (48, 17), (29, 17), (54, 27), (31, 21), (23, 19), (47, 22), (58, 26), (32, 25), (9, 9), (8, 18), (35, 26), (9, 19), (24, 21), (12, 8), (13, 13), (37, 16), (49, 21), (54, 18), (50, 18), (31, 18), (43, 23), (17, 24), (56, 13), (45, 21), (10, 9), (34, 26), (58, 15), (9, 20), (24, 28), (14, 19), (12, 7), (10, 19), (25, 25), (13, 6), (48, 15), (49, 14), (54, 21), (50, 13), (55, 16), (40, 11), (54, 15), (55, 14), (20, 28), (9, 21), (43, 12), (32, 23), (46, 11), (18, 22), (57, 17), (34, 21), (9, 29), (10, 26), (34, 15), (15, 19), (11, 19), (26, 25), (49, 7), (39, 12), (49, 25), (20, 27), (16, 11), (21, 26), (34, 16), (56, 27), (57, 26), (47, 11), (43, 27), (58, 17), (37, 11), (12, 19), (36, 16), (13, 18), (37, 21), (52, 19), (38, 26), (53, 18), (49, 18), (30, 12), (54, 25), (31, 23), (46, 21), (47, 16), (32, 27), (44, 27), (33, 26), (9, 17), (24, 23), (12, 10), (10, 22), (25, 22), (13, 11), (53, 27), (49, 11), (54, 16), (50, 16), (55, 21), (19, 20), (7, 9), (32, 18), (56, 15), (9, 26), (14, 17), (10, 17), (15, 20), (26, 28), (38, 20), (27, 17), (49, 12), (50, 11), (55, 18), (17, 23), (22, 28), (46, 9), (18, 20), (33, 20), (23, 25), (47, 12), (45, 12), (57, 29), (59, 25), (22, 25), (10, 24), (34, 13), (35, 16), (51, 21), (28, 17), (52, 22), (53, 23), (30, 11), (57, 18), (40, 26), (41, 27), (56, 29), (23, 22), (47, 21), (59, 19), (8, 9), (11, 10), (24, 18), (12, 13), (36, 18), (25, 19), (37, 19), (48, 21), (53, 16), (49, 16), (31, 11), (7, 18), (31, 17), (42, 27), (47, 18), (21, 11), (44, 21), (57, 11), (10, 10), (9, 23), (24, 25), (14, 12), (10, 20), (25, 20), (13, 9), (48, 12), (53, 25), (27, 26), (49, 9), (39, 26), (54, 22), (50, 14), (55, 13), (16, 17), (21, 20), (43, 11), (32, 20), (56, 17), (44, 12), (33, 17), (57, 12), (58, 11), (9, 24), (25, 29), (26, 26), (51, 14), (50, 9), (48, 25), (53, 12), (21, 29), (17, 21), (44, 11), (42, 23), (23, 27), (47, 14), (45, 10), (34, 17), (58, 18), (35, 12), (34, 11), (35, 18), (36, 21), (48, 16), (29, 18), (53, 21), (22, 19), (54, 26), (31, 12), (46, 22), (32, 24), (59, 16), (35, 27), (24, 20), (14, 11), (37, 17), (48, 23), (49, 22), (50, 21), (19, 19), (31, 19), (20, 20), (42, 25), (43, 20), (17, 25), (56, 12), (45, 22), (59, 15), (24, 27), (14, 18), (12, 6), (36, 27), (10, 18), (25, 26), (15, 11), (13, 7), (37, 26), (48, 14), (27, 28), (49, 15), (50, 12), (55, 17), (41, 11), (31, 24), (55, 15), (20, 19), (16, 19), (17, 18), (32, 22), (56, 19), (46, 10), (18, 25), (23, 28), (34, 20), (10, 29), (34, 14), (50, 7), (41, 12), (20, 26), (42, 11), (21, 27), (17, 11), (56, 26), (46, 13), (42, 21), (23, 21), (45, 8), (43, 24), (58, 16), (35, 14), (37, 12), (11, 9), (35, 20), (12, 18), (13, 19), (52, 18), (48, 18), (53, 19), (49, 19), (54, 24), (30, 17), (19, 12), (17, 12), (31, 20), (18, 11), (57, 28), (47, 17), (32, 26), (44, 26), (59, 18), (33, 27), (8, 19), (9, 18), (24, 22), (12, 9), (25, 23), (15, 12), (13, 12), (53, 28), (49, 20), (54, 19), (50, 19), (55, 26), (43, 22), (32, 17), (56, 14), (57, 15), (34, 27), (58, 12), (9, 27), (24, 29), (57, 27), (25, 24), (51, 13), (49, 13), (50, 10), (55, 19), (53, 15), (40, 12), (54, 12), (19, 26), (31, 26), (20, 29), (56, 21), (46, 8), (18, 23), (33, 21), (57, 16), (47, 13), (34, 18), (35, 11), (9, 28), (36, 12), (10, 27), (34, 12), (15, 18), (11, 18), (35, 17), (56, 22), (52, 21), (16, 20), (49, 24), (30, 10), (16, 12), (21, 25), (56, 28), (42, 19), (57, 25), (43, 26), (23, 18), (36, 11), (24, 17), (12, 12), (36, 17), (13, 17), (37, 20), (38, 27), (53, 17), (49, 17), (30, 13), (50, 22), (7, 19), (31, 22), (42, 26), (21, 12), (43, 19), (58, 25), (24, 24), (12, 11), (10, 23), (25, 21), (13, 10), (48, 11), (53, 26), (27, 27), (49, 10), (39, 27), (54, 17), (50, 17), (54, 11), (32, 19), (22, 26), (33, 18), (9, 25), (26, 29), (38, 21), (50, 8), (48, 24), (23, 29), (55, 11), (42, 12), (17, 22), (22, 29), (46, 14), (18, 21), (42, 22), (47, 15), (45, 11), (9, 10), (35, 13), (10, 25), (38, 12), (36, 20), (28, 18), (53, 22), (31, 13), (40, 27), (30, 18), (19, 11), (41, 26), (22, 20), (20, 12), (18, 12), (23, 17), (8, 10), (24, 19), (56, 16), (36, 19), (37, 18), (22, 27), (48, 22), (49, 23), (54, 28), (50, 20), (31, 10), (55, 25), (20, 11), (42, 24), (43, 21), (56, 11), (44, 22), (9, 22), (24, 26), (36, 26), (10, 21), (25, 27), (13, 8), (37, 27), (48, 13), (53, 24), (27, 29), (49, 8), (54, 23), (50, 15), (55, 22), (19, 25), (7, 10), (31, 25), (55, 12), (16, 18), (21, 19), (17, 19), (32, 21), (56, 18), (18, 24), (57, 19), (10, 28), (25, 28), (15, 17), (23, 20), (11, 17), (26, 27), (38, 11), (27, 18), (53, 11)]



came_from = {(28, 28): (27, 27), (11, 11): (10, 10), (12, 12): (11, 11), (26, 28): (27, 27), (47, 28): (46, 29), (18, 19): (18, 18), (29, 25): (28, 26), (28, 26): (27, 27), (19, 19): (18, 18), (10, 8): (9, 9), (34, 29): (34, 28), (6, 7): (6, 6), (5, 5): (4, 4), (7, 6): (6, 6), (28, 24): (27, 25), (25, 26): (25, 25), (41, 29): (40, 29), (24, 24): (23, 23), (21, 23): (22, 22), (20, 21): (20, 20), (27, 28): (27, 27), (1, 1): (0, 0), (23, 25): (24, 24), (0, 1): (0, 0), (3, 2): (2, 2), (20, 20): (19, 19), (24, 23): (23, 23), (49, 28): (48, 29), (4, 5): (4, 4), (31, 25): (30, 25), (7, 5): (6, 6), (14, 15): (14, 14), (40, 28): (39, 29), (12, 11): (11, 11), (20, 18): (19, 19), (29, 24): (28, 25), (15, 14): (14, 14), (37, 29): (36, 29), (6, 8): (7, 7), (17, 18): (17, 17), (39, 29): (38, 29), (27, 27): (26, 26), (32, 25): (31, 25), (3, 1): (2, 2), (9, 9): (8, 8), (16, 18): (17, 17), (20, 19): (19, 19), (21, 19): (20, 20), (15, 16): (15, 15), (46, 29): (45, 29), (36, 29): (35, 29), (26, 24): (25, 25), (16, 16): (15, 15), (22, 23): (22, 22), (13, 13): (12, 12), (23, 22): (22, 22), (2, 1): (1, 1), (43, 29): (42, 29), (8, 9): (8, 8), (17, 19): (18, 18), (24, 22): (23, 23), (11, 10): (10, 10), (42, 28): (41, 29), (12, 13): (12, 12), (26, 29): (27, 28), (45, 28): (44, 29), (33, 29): (34, 28), (18, 16): (17, 17), (23, 21): (22, 22), (28, 27): (27, 27), (2, 2): (1, 1), (29, 27): (28, 28), (8, 6): (7, 7), (25, 23): (24, 24), (29, 26): (28, 27), (10, 9): (9, 9), (9, 7): (8, 8), (6, 4): (5, 5), (5, 4): (4, 4), (19, 17): (18, 18), (24, 28): (25, 27), (11, 9): (10, 10), (21, 20): (20, 20), (41, 28): (40, 29), (25, 25): (24, 24), (1, 0): (0, 0), (23, 24): (23, 23), (3, 5): (4, 4), (33, 28): (33, 27), (4, 6): (5, 5), (10, 10): (9, 9), (7, 8): (7, 7), (5, 7): (6, 6), (24, 25): (24, 24), (14, 12): (13, 13), (38, 28): (37, 29), (37, 28): (36, 29), (17, 17): (16, 16), (27, 26): (26, 26), (1, 3): (2, 2), (32, 26): (31, 25), (28, 29): (28, 28), (9, 8): (8, 8), (33, 27): (32, 26), (15, 13): (14, 14), (35, 29): (34, 28), (16, 14): (15, 15), (48, 28): (47, 29), (26, 25): (25, 25), (17, 15): (16, 16), (16, 17): (16, 16), (22, 20): (21, 21), (13, 12): (12, 12), (32, 28): (33, 27), (19, 20): (19, 19), (27, 25): (26, 26), (43, 28): (42, 29), (8, 10): (9, 9), (28, 25): (27, 26), (9, 11): (10, 10), (7, 9): (8, 8), (30, 24): (29, 25), (11, 13): (12, 12), (25, 29): (26, 28), (12, 14): (13, 13), (26, 26): (25, 25), (13, 15): (14, 14), (18, 17): (17, 17), (24, 26): (25, 25), (21, 21): (20, 20), (2, 3): (2, 2), (8, 7): (7, 7), (39, 28): (38, 29), (4, 2): (3, 3), (44, 28): (43, 29), (34, 27): (33, 27), (6, 5): (5, 5), (5, 3): (4, 4), (24, 29): (25, 28), (14, 16): (15, 15), (25, 24): (24, 24), (34, 26): (33, 27), (18, 20): (19, 19), (21, 22): (21, 21), (47, 29): (46, 29), (18, 18): (17, 17), (40, 29): (39, 29), (19, 21): (20, 20), (3, 4): (3, 3), (2, 4): (3, 3), (29, 28): (28, 28), (10, 11): (10, 10), (34, 28): (33, 27), (6, 6): (5, 5), (5, 6): (5, 5), (7, 7): (6, 6), (14, 13): (13, 13), (25, 27): (26, 26), (17, 16): (16, 16), (42, 29): (41, 29), (20, 22): (21, 21), (1, 2): (1, 1), (32, 27): (32, 26), (29, 29): (28, 28), (3, 3): (2, 2), (33, 26): (32, 26), (49, 29): (48, 29), (4, 4): (3, 3), (10, 12): (11, 11), (35, 28): (34, 28), (16, 15): (15, 15), (14, 14): (13, 13), (12, 10): (11, 11), (27, 29): (28, 28), (15, 15): (14, 14), (22, 21): (21, 21), (13, 11): (12, 12), (0, 0): None, (22, 24): (23, 23), (38, 29): (37, 29), (27, 24): (26, 25), (32, 24): (31, 25), (48, 29): (47, 29), (19, 18): (18, 18), (33, 25): (32, 26), (9, 10): (9, 9), (24, 27): (25, 26), (45, 29): (44, 29), (30, 25): (29, 26), (11, 12): (11, 11), (35, 27): (34, 28), (25, 28): (26, 27), (15, 17): (16, 16), (46, 28): (45, 29), (36, 28): (35, 29), (26, 27): (26, 26), (22, 22): (21, 21), (13, 14): (13, 13), (23, 23): (22, 22), (0, 2): (1, 1), (2, 0): (1, 1), (8, 8): (7, 7), (4, 3): (3, 3), (44, 29): (43, 29), (31, 24): (30, 25)}

# whole_map = []
# for x in range(60):
#     for y in range(30):
#         point = [(x, y)]
#         whole_map += point
# print 'whole_map: \n', whole_map


draw_grid(diagram, width=1, point_to=came_from, start=(0,0), goal=(45, 13))