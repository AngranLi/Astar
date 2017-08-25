import random
import timeit
import rospy
import inc

class Cell():
    def __init__(self):
        ''' 4 pointers '''
        self.move = {}
        self.succ = {}
        self.searchtree = {}
        self.trace = {}

        self.obstacle = 0
        self.x = 0
        self.y = 0
        self.z = 0
        # self.dfsx = 0   # needed only for generating dfs mazes
        # self.dfsy = 0
        # self.dfsz = 0
        self.g = inc.LARGE
        self.rhs = inc.LARGE
        self.key = [0, 0, 0]
        self.generated = 0
        self.heapindex = 0

class Obstacle():
    def __init__(self, name, centrePoint, length=inc.MAZELENGTH/4, width=inc.MAZEWIDTH/4, height=inc.MAZEHEIGHT):
        self.centrePoint = centrePoint
        self.length = length
        self.width  = width
        self.height = height
        self.name = name

        self.left    = max(0, self.centrePoint[0]-self.length/2)
        self.right   = min(self.centrePoint[0]+self.length/2, inc.MAZELENGTH-1)
        self.back    = max(0, self.centrePoint[1]-self.width/2)
        self.front   = min(self.centrePoint[1]+self.width/2, inc.MAZEWIDTH-1)
        self.bottom  = max(0, self.centrePoint[2]-self.height/2)
        self.top     = min(self.centrePoint[2]+self.height/2, inc.MAZEHEIGHT-1)

    # def updateRange():
    #     self.left    = max(0, self.centrePoint[0]-self.length/2)
    #     self.right   = min(self.centrePoint[0]+self.length/2, inc.MAZELENGTH-1)
    #     self.back    = max(0, self.centrePoint[1]-self.width/2)
    #     self.front   = min(self.centrePoint[1]+self.width/2, inc.MAZEWIDTH-1)
    #     self.bottom  = max(0, self.centrePoint[2]-self.height/2)
    #     self.top     = min(self.centrePoint[2]+self.height/2, inc.MAZEHEIGHT-1)


def preprocessmaze():
    global maze
    global mazestart
    global mazegoal
    global mazeiteration

    mazeiteration = 0

    goaly = inc.GOALY
    goalx = inc.GOALX
    goalz = inc.GOALZ
    starty = inc.STARTY
    startx = inc.STARTX
    startz = inc.STARTZ

    # startTime = timeit.default_timer()
    print ' initialising environment...'
    maze = []
    for x in range(inc.MAZELENGTH):
        column1 = []
        for y in range(inc.MAZEWIDTH):
            column2 = []
            for z in range(inc.MAZEHEIGHT):
                cell = Cell()
                cell.x = x
                cell.y = y
                cell.z = z
                column2.append(cell)
            column1.append(column2)
        maze.append(column1)
    # initEnvtime = timeit.default_timer() - startTime
    # print 'time for initialising environment: ', initEnvtime

    # startTime = timeit.default_timer()
    print ' adding successors...'
    for x in range(inc.MAZELENGTH):
        for y in range(inc.MAZEWIDTH):
            for z in range(inc.MAZEHEIGHT):
                for d in range(inc.DIRECTIONS):
                    newx = x + inc.dx[d]
                    newy = y + inc.dy[d]
                    newz = z + inc.dz[d]
                    if (0<=newx<inc.MAZELENGTH and 0<=newy<inc.MAZEWIDTH and 0<=newz<inc.MAZEHEIGHT):
                        maze[x][y][z].succ[d] = maze[newx][newy][newz]
                    else:
                        maze[x][y][z].succ[d] = None
                    maze[x][y][z].move[d] = maze[x][y][z].succ[d]
    # addSuccTime = timeit.default_timer() - startTime
    # print 'time for adding successors: ', addSuccTime
    # #ifdef DEBUG
    #     assert(STARTY % 2 == 0);
    #     assert(STARTX % 2 == 0);
    #     assert(GOALY % 2 == 0);
    #     assert(GOALX % 2 == 0);
    # #endif

    mazestart = maze[inc.STARTY][inc.STARTX][inc.STARTZ]
    mazegoal  = maze[inc.GOALY][inc.GOALX][inc.GOALZ]
    mazeiteration = 0


def postprocessmaze():
    # global maze
    # global mazestart
    global mazegoal
    tmpcell = Cell()

    # startTime = timeit.default_timer()
    # for x in range(inc.MAZELENGTH):
    #     for y in range (inc.MAZEWIDTH):
    #         for z in range (inc.MAZEHEIGHT):
    #             for d1 in range(inc.DIRECTIONS):
    #                 maze[x][y][z].move[d1] = maze[x][y][z].succ[d1]
    # addMoveTime = timeit.default_timer() - startTime
    # print 'time for adding move: ', addMoveTime

    # startTime = timeit.default_timer()
    '''don't know the purpose of this section'''
    for d1 in range(inc.DIRECTIONS):
        if mazegoal.move[d1] and mazegoal.move[d1].obstacle:
            tmpcell = mazegoal.move[d1]
            for d2 in range(inc.DIRECTIONS):
                if tmpcell.move[d2]:
                    tmpcell.move[d2] = None
                    tmpcell.succ[d2].move[inc.reverse[d2]] = None
    # obstReverseTime = timeit.default_timer() - startTime
    # print 'time for reverse obst: ', obstReverseTime


def newEmptyEnv():
    global maze
    global mazestart
    global mazegoal
    global obstDict
    tmpcell = Cell()

    # startTime = timeit.default_timer()
    print 'pre-processing maze...'
    preprocessmaze()
    # preProcessTime = timeit.default_timer() - startTime
    # print 'time for pre-processing: ', preProcessTime

    print 'adding obstacles...'
    obstDict = {}
    # for i in range(3):
    #     tempctpt = (random.randint(int(0.1*inc.MAZELENGTH),int(0.8*inc.MAZELENGTH)),
    #                     random.randint(int(0.1*inc.MAZEWIDTH),int(0.8*inc.MAZEWIDTH)),
    #                         random.randint(int(0.1*inc.MAZEHEIGHT),int(0.8*inc.MAZEHEIGHT)))
    #     obstDict['obst'+str(i)] = Obstacle('obst'+str(i), tempctpt)
    #     updateObst(obstDict['obst'+str(i)])
    mazegoal.obstacle = 0

    # startTime = timeit.default_timer()
    print 'post-processing maze...'
    postprocessmaze()
    # postProcessTime = timeit.default_timer() - startTime
    # print 'time for post-processing: ', postProcessTime

def updateObst(tempobst):
    global obstDict

    for key in obstDict.copy():
        if tempobst.name == key:
            for x in range(obstDict[key].left, obstDict[key].right+1):
                for y in range(obstDict[key].back, obstDict[key].front+1):
                    maze[x][y][obstDict[key].bottom].obstacle = 0
                    maze[x][y][obstDict[key].top].obstacle = 0
            for x in range(obstDict[key].left, obstDict[key].right+1):
                for z in range(obstDict[key].bottom, obstDict[key].top+1):
                    maze[x][obstDict[key].back][z].obstacle = 0
                    maze[x][obstDict[key].front][z].obstacle = 0
            for y in range(obstDict[key].back, obstDict[key].front+1):
                for z in range(obstDict[key].bottom, obstDict[key].top+1):
                    maze[obstDict[key].left][y][z].obstacle = 0
                    maze[obstDict[key].right][y][z].obstacle = 0

            for x in range(tempobst.left, tempobst.right+1):
                for y in range(tempobst.back, tempobst.front+1):
                    maze[x][y][tempobst.bottom].obstacle = 1
                    maze[x][y][tempobst.top].obstacle = 1
            for x in range(tempobst.left, tempobst.right+1):
                for z in range(tempobst.bottom, tempobst.top+1):
                    maze[x][tempobst.back][z].obstacle = 1
                    maze[x][tempobst.front][z].obstacle = 1
            for y in range(tempobst.back, tempobst.front+1):
                for z in range(tempobst.bottom, tempobst.top+1):
                    maze[tempobst.left][y][z].obstacle = 1
                    maze[tempobst.right][y][z].obstacle = 1
            obstDict[tempobst.name] = tempobst
    else:
        obstDict[tempobst.name] = tempobst
        for x in range(tempobst.left, tempobst.right+1):
            for y in range(tempobst.back, tempobst.front+1):
                maze[x][y][tempobst.bottom].obstacle = 1
                maze[x][y][tempobst.top].obstacle = 1
        for x in range(tempobst.left, tempobst.right+1):
            for z in range(tempobst.bottom, tempobst.top+1):
                maze[x][tempobst.back][z].obstacle = 1
                maze[x][tempobst.front][z].obstacle = 1
        for y in range(tempobst.back, tempobst.front+1):
            for z in range(tempobst.bottom, tempobst.top+1):
                maze[tempobst.left][y][z].obstacle = 1
                maze[tempobst.right][y][z].obstacle = 1
    '''
    startTime = timeit.default_timer()
    for x in range(inc.MAZELENGTH):
        for y in range(inc.MAZEWIDTH):
            for z in range(inc.MAZEHEIGHT):
                maze[0][1][2].obstacle = 0
    clearObstTime = timeit.default_timer() - startTime
    print 'time for clearing obst: ', clearObstTime
    '''

    # startTime = timeit.default_timer()
    # addObstTime = timeit.default_timer() - startTime
    # print 'time for adding obstacles: ', addObstTime


'''
def newdfsmaze(wallstoremove):
    global maze
    global mazestart
    global mazegoal
    global mazeiteration
    permute = [0, 1, 2, 3, 4, 5, 6, 7]

    preprocessmaze()

    #ifdef DEBUG
        assert(MAZELENGTH % 2 == 1);
        assert(MAZEWIDTH % 2 == 1);
    #endif

    for y in range (inc.MAZEWIDTH):
        for x in range(inc.MAZELENGTH):
    	    maze[y][x].obstacle = 1
    	    maze[y][x].dfsx = 0
    	    maze[y][x].dfsy = 0
    x = 0
    y = 0
    maze[y][x].dfsx = -1
    maze[y][x].dfsy = -1
    while True:
        if maze[y][x].obstacle:
            maze[y][x].obstacle = 0
        for d in range(inc.DIRECTIONS-1):
            randomnumber = random.randint(0, inc.DIRECTIONS-d-1)
            permutetmp = permute[randomnumber]
            permute[randomnumber] = permute[inc.DIRECTIONS-1-d]
            permute[inc.DIRECTIONS-1-d] = permutetmp

        dtmp = 0
        while dtmp<inc.DIRECTIONS:
            d = permute[dtmp]
            newx = x + 2*inc.dx[d]
            newy = y + 2*inc.dy[d]
            # print 'x/y: ', x, y
            # print 'succ:', maze[y][x].succ
            # if (maze[y][x].succ[d]):
            #     print 'y/x: ', y, x
            #     print maze[newy][newx].obstacle
                # rospy.signal_shutdown()
            if (maze[y][x].succ[d]) and (maze[newy][newx].obstacle):
                if (maze[y + inc.dy[d]][x + inc.dx[d]].obstacle):
                    maze[y + inc.dy[d]][x + inc.dx[d]].obstacle = 0
                maze[newy][newx].dfsx = x
                maze[newy][newx].dfsy = y
                x = newx
                y = newy
                break
            dtmp += 1

        if dtmp == inc.DIRECTIONS:
            # print 'in loop'
            if maze[y][x].dfsx == -1:
                break
            newx = maze[y][x].dfsx
            newy = maze[y][x].dfsy
            x = newx
            y = newy

    while wallstoremove > 0:
    	newx = random.randint(0, inc.MAZELENGTH-1)
    	newy = random.randint(0, inc.MAZEWIDTH-1)
    	if (maze[newy][newx].obstacle):
    	    maze[newy][newx].obstacle = 0
    	    wallstoremove -= 1

    mazegoal.obstacle = 0

    # if STARTCANBEBLOCKED:
    #     mazestart.obstacle = 0

    postprocessmaze()
'''
