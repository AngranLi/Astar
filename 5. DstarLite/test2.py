'''
import inc
import initMaze
# i = 0
#
# def temp():
#     global i
#     i = 1
#
# def showit():
#     global i
#     return i
#
# temp()

# lst2 = {}
#
# def showit():
#     print 'length of test2.lst2: ', len(lst2)
#
# def modnfoo(nfoo):
#     nfoo = 5
#
# def modlstfoo(lstfoo):
#     lstfoo[0] = 6
#
# def moddictfoo(dictfoo):
#     dictfoo[1] = 'A'
#
# p = [7, 7]

def preprocessmaze():
    global maze
    global mazestart
    global mazegoal
    global mazeiteration
    # if (maze == NULL)

    maze = []
    for y in range(5):
        column = []
        for x in range(5):
            cell = initMaze.Cell()
            cell.x = x
            cell.y = y
            for d in range(4):
                newy = y + inc.dy[d]
                newx = x + inc.dx[d]
                if (newy >= 0 and newy <= 5 and newx >= 0 and newx <= 5):
                    cell.succ[d] = (newy,newx)
                else:
                    cell.succ[d] = None
            column.append(cell)
        maze.append(column)

maze = []
preprocessmaze()

cell2 = maze[2][2]
'''

def keyless():
    for i in range (keylength):
        print i

def test2():
    global keylength
    keylength = 2

keylength = 3