# length_of_map = 10
# width_of_map  = 10
# height_of_map = 5
#
# obst = Obstacle([2, 2, 2], 'obst_UAV')
# obst.setSize([4, 2, 2])
# print obst.range
# print
#
# point = (1,1,6)
# print obst.conflict(point)
# print
# print obst.points

# number1 = 2.1
# number2 = 2.4
# number3 = 2.5
# number4 = 2.9
#
# print int(number1)
# print int(number2)
# print int(number3)
# print int(number4)


# def gridalize(value, scale):
#     if isinstance(value, tuple):
#         result = []
#         value = list(value)
#         for i in range(len(value)):
#             result.append(int(round(value[i] *scale)))
#         return tuple(result)
#     else:
#         return int(round(value *scale))
#
#
#
# (l, w, h) = gridalize((1.5, 2.8, 3), 3)
# print l
# print w
# print h


# lst = [[1,2,3], [2,4,5]]
# for point in lst:
#     print point


# dct = {(2, 2, 2): (4, 4, 4), (3, 3, 3): (5, 5, 5)}
# f = open('test.txt', 'w')
# for key in dct:
#     string = str(key) + ':' + str(dct[key]) + '  '
#     f.write(string)
# f.close()


class SquareGrid(object):
    def __init__(self, length, width, height):
        self.length = length
        self.width  = width
        self.height = height
        self.walls = []

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
        global obstArray
        results = filter(self.passable, results, obstArray)
        return results


class GridWithWeights(SquareGrid):
    def __init__(self, length, width, height):
        super(GridWithWeights, self).__init__(length, width, height)
        self.weights = {}

    def cost(self, from_node, to_node):
        return self.weights.get(to_node, 1) # default is 1
