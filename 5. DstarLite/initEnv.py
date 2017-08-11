import random
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

    print 'initialising environment...'
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

    print 'adding successors...'
    for x in range(inc.MAZELENGTH):
        for y in range(inc.MAZEWIDTH):
            for z in range(inc.MAZEHEIGHT):
                for d in range(inc.DIRECTIONS):
                    newy = y + inc.dy[d]
                    newx = x + inc.dx[d]
                    newz = z + inc.dz[d]
                    if (0<=newx<inc.MAZELENGTH and 0<=newy<inc.MAZEWIDTH and 0<=newz<inc.MAZEHEIGHT):
                        maze[x][y][z].succ[d] = maze[newx][newy][newz]
                    else:
                        maze[x][y][z].succ[d] = None
    # #ifdef DEBUG
    #     assert(STARTY % 2 == 0);
    #     assert(STARTX % 2 == 0);
    #     assert(GOALY % 2 == 0);
    #     assert(GOALX % 2 == 0);
    # #endif

    mazestart   = maze[inc.STARTY][inc.STARTX][inc.STARTZ]
    mazegoal    = maze[inc.GOALY][inc.GOALX][inc.GOALZ]
    mazeiteration = 0


def postprocessmaze():
    global maze
    global mazestart
    global mazegoal
    tmpcell = Cell()

    for x in range(inc.MAZELENGTH):
        for y in range (inc.MAZEWIDTH):
            for z in range (inc.MAZEHEIGHT):
                for d1 in range(inc.DIRECTIONS):
                    maze[x][y][z].move[d1] = maze[x][y][z].succ[d1]

    '''don't know the purpose of this section'''
    for d1 in range(inc.DIRECTIONS):
        if mazegoal.move[d1] and mazegoal.move[d1].obstacle:
            tmpcell = mazegoal.move[d1]
            for d2 in range(inc.DIRECTIONS):
                if tmpcell.move[d2]:
                    tmpcell.move[d2] = None
                    tmpcell.succ[d2].move[inc.reverse[d2]] = None


def newrandommaze():
    global maze
    global mazestart
    global mazegoal
    tmpcell = Cell()

    print 'pre-processing maze...'
    preprocessmaze()

    print 'adding obstacles...'
    obstCentrePoint = []
    obstLength = 10
    obstWidth = 10
    obstHeight = 5
    for i in range(1):
        obstCentrePoint.append((random.randint(int(0.1*inc.MAZELENGTH),int(0.8*inc.MAZELENGTH)), random.randint(int(0.1*inc.MAZEWIDTH),int(0.8*inc.MAZEWIDTH)), random.randint(int(0.1*inc.MAZEHEIGHT),int(0.8*inc.MAZEHEIGHT))))
        for x in range(obstCentrePoint[i][0]-obstLength/2, obstCentrePoint[i][0]+obstLength/2):
            for y in range(obstCentrePoint[i][1]-obstWidth/2, obstCentrePoint[i][1]+obstWidth/2):
                for z in range(obstCentrePoint[i][2]-obstHeight/2, obstCentrePoint[i][2]+obstHeight/2):
                    maze[x][y][z].obstacle = 1
    # for y in range (inc.MAZEWIDTH):
    #     for x in range(inc.MAZELENGTH):
	   #         maze[y][x].obstacle = (random.randint(1,10000) < 10000 * inc.MAZEDENSITY)
    mazegoal.obstacle = 0
    # if inc.STARTCANBEBLOCKED:
    #     mazestart.obstacle = 0

    print 'post-processing maze...'
    postprocessmaze()


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
