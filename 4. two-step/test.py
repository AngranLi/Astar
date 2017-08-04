import timeit
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


# a = (1, 2, 3)
# b = (4, 5, 6)
# print timeit.timeit('(4-1)**2 + (5-2)**2', number = 100000)

# def divideNumbers(num1, num2):
#     return num1 % num2 == 0
#
# lst = range(10)
# print 'lst: ', lst
# print filter(lambda x: divideNumbers(x, 2), lst)

# dictionary = {(1,1): (1,2), (1,3): (1,2)}
# print len(dictionary)


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

obst1 = Obstacle((1,1,1), 0.25, 2, 30, 'obst_UAV')
obst2 = Obstacle((2,2,2), 0.5, 2, 30, 'obst_UGV')
dictionary = {'obst1':obst1, 'obst2':obst2}
# for key in dictionary:
#     if key == 'obst1':
#         print 'yeah'
#         print dictionary[key]

print type([obst1, obst2])
print type(dictionary.values())
