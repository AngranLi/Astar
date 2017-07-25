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

def divideNumbers(num1, num2):
    return num1 % num2 == 0

lst = range(10)
print 'lst: ', lst
print filter(lambda x: divideNumbers(x, 2), lst)
