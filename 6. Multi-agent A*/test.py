# lst = [0, 1, 2]
# for i in range(5):
#     try:
#         print lst[i]
#     except IndexError:
#         print 'i is ', i
#
# print 'last elemnt: ', lst[-1]

import math

for i in range(5, -1, -1):
    try:
        print 5/float(i)
    except ZeroDivisionError:
        print 'i is ', i
