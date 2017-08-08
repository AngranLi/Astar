import test2
'''
import random
# import inc
import initMaze
import initHeap
# def preprocessmaze():
#     global maze
#     global mazestart
#     global mazegoal
#     global mazeiteration
#     # if (maze == NULL)
#
#     maze = []
#     for y in range(5):
#         column = []
#         for x in range(5):
#             cell = initMaze.Cell()
#             cell.x = x
#             cell.y = y
#             for d in range(4):
#                 newy = y + inc.dy[d]
#                 newx = x + inc.dx[d]
#                 if (newy >= 0 and newy <= 5 and newx >= 0 and newx <= 5):
#                     cell.succ[d] = (newy,newx)
#                 else:
#                     cell.succ[d] = None
#             column.append(cell)
#         maze.append(column)



# nfoo = 1
# lstfoo = [1,2]
# dictfoo = {1:1, 2:2}
# test2.modnfoo(nfoo)
# test2.modlstfoo(lstfoo)
# test2.moddictfoo(dictfoo)
# print nfoo
# print lstfoo
# dictfoo[1] = 'A'
# print dictfoo[1].key()
# print dictfoo

cell1 = initMaze.Cell()
cell2 = initMaze.Cell()
cell2.y = 7
print initHeap.heap
initHeap.insertheap(cell1)
print initHeap.heap
cell1 = initMaze.Cell()
initHeap.insertheap(cell1)
print initHeap.heap
cell1 = initMaze.Cell()
initHeap.insertheap(cell1)
print initHeap.heap
cell1 = initMaze.Cell()
initHeap.insertheap(cell1)
print initHeap.heap

# def updatekey(tmpcell):
#     tmpcell.key = ['A', 'B', 'C']
#
# def updaterhs(tmpcell):
#     tmpcell.rhs = 777
#
# def updatesearchtree(tmpcell):
#     tmpcell.searchtree = {'A':(1,2)}
#
# cell1 = initMaze.Cell()
# cell1 = test2.cell2
# print test2.maze[2][2].searchtree
# cell1.searchtree = None
# print test2.maze[2][2].searchtree
#
# updaterhs(cell1)
# updatekey(cell1)
# updatesearchtree(cell1)
# print test2.maze[2][2].searchtree
'''


dictfoo = {1:1}
dictfoo[1] = 2
print dictfoo